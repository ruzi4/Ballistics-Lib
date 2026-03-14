// Ballistics Trajectory Renderer
// Interactive 3D application that places a launcher and target in a scene,
// computes a fire solution in real time, and draws the resulting trajectory.
//
// Coordinate conventions
//   Ballistics lib : x = East, y = North, z = Up  (right-handed, z-up)
//   raylib         : x = East, y = Up,    z = South (right-handed, y-up)
//   Transform      : rl = { ball.x, ball.z, -ball.y }
//
// Controls
//   W/S/A/D           – move target North/South/East/West  (disabled when moving target active)
//   Q / E (key)       – lower / raise target altitude      (disabled when moving target active)
//   Arrow keys        – move launcher North/South/East/West
//   Page Up / Page Dn – raise / lower launcher altitude
//   Right-mouse drag  – orbit camera
//   Middle-mouse drag – pan camera (Free mode only)
//   Scroll wheel      – zoom

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <ballistics/ballistics.hpp>
#include <ballistics/math/math_constants.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <vector>

using namespace ballistics;

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------
static inline Vector3 to_rl(const Vec3& v)
{
    return { (float)v.x, (float)v.z, -(float)v.y };
}

// ---------------------------------------------------------------------------
// Data shared between background solver and render thread
// ---------------------------------------------------------------------------
struct SolveResult {
    bool   valid         = false;
    double azimuth_deg   = 0.0;
    double elevation_deg = 0.0;
    double flight_time_s = 0.0;
    double range_m       = 0.0;
    double max_range_m   = 0.0;  // max achievable range (for diagnostics)
    double alt_diff_m    = 0.0;  // launcher altitude - target altitude
    std::vector<Vector3> traj;   // trajectory points in raylib coords
};

struct SolveParams {
    Vec3                 launcher_pos;
    Vec3                 target_pos;
    MunitionSpec         munition;
    AtmosphericConditions atmo;
    double               muzzle_speed;
};

// ---------------------------------------------------------------------------
// Background: compute fire solution + collect full trajectory arc
// ---------------------------------------------------------------------------
static SolveResult solve_async(const SolveParams& p)
{
    SolveResult out;

    const double dx      = p.target_pos.x - p.launcher_pos.x;
    const double dy      = p.target_pos.y - p.launcher_pos.y;
    const double range_m = std::sqrt(dx * dx + dy * dy);
    out.range_m = range_m;

    out.alt_diff_m = p.launcher_pos.z - p.target_pos.z;

    if (range_m < 1.0) return out; // launcher and target too close

    const double az_deg       = std::atan2(dx, dy) * kRadToDeg;
    const double launch_height = p.launcher_pos.z - p.target_pos.z;
    const double target_alt    = p.target_pos.z;

    TrajectorySimulator sim(p.munition, p.atmo);
    LauncherOrientation orient;
    orient.azimuth_deg = az_deg;

    // Build a quick table to find max range for diagnostics
    FireControlTable diag_table;
    diag_table.build(sim, p.muzzle_speed, az_deg, launch_height,
                     false, 50, target_alt);
    out.max_range_m = diag_table.max_range_m();

    FireSolution sol = solve_elevation(sim, orient, range_m,
                                       p.muzzle_speed,
                                       launch_height, /*high_angle=*/false,
                                       /*tolerance_m=*/0.5,
                                       target_alt);
    if (!sol.valid) return out;

    out.valid         = true;
    out.azimuth_deg   = az_deg;
    out.elevation_deg = sol.elevation_deg;
    out.flight_time_s = sol.flight_time_ms / 1000.0;

    // Collect trajectory points in absolute world coordinates
    const double az_r = az_deg           * kDegToRad;
    const double el_r = sol.elevation_deg * kDegToRad;
    const double v    = p.muzzle_speed;

    ProjectileState init;
    init.position = p.launcher_pos;
    init.velocity = {
        v * std::sin(az_r) * std::cos(el_r),   // East  (+x)
        v * std::cos(az_r) * std::cos(el_r),   // North (+y)
        v * std::sin(el_r)                      // Up    (+z)
    };
    init.time = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 60.0;                           // 60 Hz — fine for visualization
    cfg.use_rk4  = true;
    cfg.max_time = out.flight_time_s * 1.5 + 2.0;
    cfg.ground_z = p.target_pos.z - 0.5;                  // slightly below target

    auto states = sim.simulate(init, cfg);
    out.traj.reserve(states.size());
    for (const auto& s : states)
        out.traj.push_back(to_rl(s.position));

    return out;
}

// ---------------------------------------------------------------------------
// Draw launcher: box body + rotating barrel cylinder + muzzle direction line
// ---------------------------------------------------------------------------
static void draw_launcher(Vector3 pos, float az_deg, float el_deg)
{
    // Body
    DrawCube     (pos, 3.0f, 1.2f, 2.0f, { 55, 115, 55, 255 });
    DrawCubeWires(pos, 3.0f, 1.2f, 2.0f, {  0,  70,  0, 255 });

    // Firing direction in raylib coords:
    //   az=0   → -z (North),  az=90 → +x (East)
    //   el > 0 → upward (+y)
    const float az_r = az_deg * DEG2RAD;
    const float el_r = el_deg * DEG2RAD;
    Vector3 dir = {
         sinf(az_r) * cosf(el_r),   // East  (+x)
         sinf(el_r),                 // Up    (+y)
        -cosf(az_r) * cosf(el_r)    // South (+z); negated → North
    };

    // Barrel (cylinder from top-center of box outward)
    Vector3 barrel_base = { pos.x, pos.y + 0.6f, pos.z };
    Vector3 barrel_tip  = Vector3Add(barrel_base, Vector3Scale(dir, 3.5f));
    DrawCylinderEx     (barrel_base, barrel_tip, 0.14f, 0.11f, 10, { 100, 100, 100, 255 });
    DrawCylinderWiresEx(barrel_base, barrel_tip, 0.14f, 0.11f, 10, {  55,  55,  55, 255 });

    // Muzzle direction line (red ray extending from barrel tip)
    Vector3 muzzle_end = Vector3Add(barrel_tip, Vector3Scale(dir, 5.0f));
    DrawLine3D(barrel_tip, muzzle_end, RED);
}

// ---------------------------------------------------------------------------
// Draw target: sphere + vertical stake + ground crosshair
// ---------------------------------------------------------------------------
static void draw_target(Vector3 pos)
{
    DrawSphere     (pos, 0.6f, { 220, 50, 50, 255 });
    DrawSphereWires(pos, 0.6f, 8, 8, { 150, 20, 20, 255 });

    // Vertical stake to ground plane
    Vector3 ground = { pos.x, 0.02f, pos.z };
    DrawLine3D(ground, pos, { 200, 80, 80, 180 });

    // Crosshair on ground
    const float arm = 2.0f;
    const float gy  = 0.03f;
    DrawLine3D({ pos.x - arm, gy, pos.z        }, { pos.x + arm, gy, pos.z        }, { 220, 80, 80, 255 });
    DrawLine3D({ pos.x,       gy, pos.z - arm  }, { pos.x,       gy, pos.z + arm  }, { 220, 80, 80, 255 });
}

// ---------------------------------------------------------------------------
// Draw trajectory arc + apex marker
// ---------------------------------------------------------------------------
static void draw_trajectory(const std::vector<Vector3>& pts)
{
    if (pts.size() < 2) return;

    // Find apex (highest y = Up in raylib)
    size_t apex = 0;
    for (size_t i = 1; i < pts.size(); ++i)
        if (pts[i].y > pts[apex].y) apex = i;

    for (size_t i = 1; i < pts.size(); ++i)
        DrawLine3D(pts[i - 1], pts[i], { 255, 165, 0, 255 });

    // Apex marker
    if (apex > 0 && apex < pts.size() - 1)
        DrawSphere(pts[apex], 0.4f, YELLOW);
}

// ---------------------------------------------------------------------------
int main()
{
    // -----------------------------------------------------------------------
    // Window
    // -----------------------------------------------------------------------
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1420, 920, "Ballistics Trajectory Renderer");
    SetTargetFPS(60);

    // -----------------------------------------------------------------------
    // Load munitions
    // -----------------------------------------------------------------------
    MunitionLibrary lib;
    lib.load("data/munitions.json");
    auto mun_names = lib.names();

    // Build semicolon-separated string for GuiDropdownBox
    std::string dd_str;
    for (size_t i = 0; i < mun_names.size(); ++i) {
        if (i) dd_str += ';';
        dd_str += mun_names[i];
    }

    // -----------------------------------------------------------------------
    // Scene state  (metres, ballistics coords: x=East, y=North, z=Up)
    // -----------------------------------------------------------------------
    float lx = 0.f, ly = 0.f, lz = 1.5f;       // launcher position
    float tx = 500.f, ty = 300.f, tz = 0.f;     // target position

    // Moving target state
    bool  target_moving       = false;
    float target_speed        = 20.f;    // m/s
    float target_yaw          = 0.f;     // degrees, 0=North, 90=East
    float target_travel_dist  = 200.f;   // metres
    float target_travel_prog  = 0.f;     // current progress (0..dist), ping-pong
    bool  target_travel_fwd   = true;    // ping-pong direction
    float target_origin_x     = tx;      // captured on toggle-on
    float target_origin_y     = ty;
    bool  prev_target_moving  = false;

    int  mun_idx  = 1;    // 5.56×45 M855 as default
    float muzzle_speed = (float)lib.get(mun_names[(size_t)mun_idx]).muzzle_velocity_ms;
    bool dd_edit  = false;

    // Auto-update muzzle speed when munition changes
    int last_mun_for_speed = mun_idx;

    // -----------------------------------------------------------------------
    // Async fire-solution state
    // -----------------------------------------------------------------------
    std::future<SolveResult> pending;
    SolveResult current;
    bool computing = false;
    bool dirty     = true;

    // Previous values — used to detect changes and trigger recompute
    float p_lx = lx - 1.f, p_ly = ly, p_lz = lz;
    float p_tx = tx,        p_ty = ty, p_tz = tz;
    int   p_mi = mun_idx - 1;
    float p_mv = muzzle_speed - 1.f;

    // -----------------------------------------------------------------------
    // Camera orbit state
    // -----------------------------------------------------------------------
    float cam_dist = 280.f;
    float cam_az   = 30.f;   // horizontal orbit angle (degrees)
    float cam_el   = 35.f;   // vertical elevation (degrees)
    float pan_x    = 0.f;    // pan offset in raylib x (East)
    float pan_z    = 0.f;    // pan offset in raylib z (South)

    // View focus: 0 = Free (midpoint + pan), 1 = Launcher, 2 = Target
    int  cam_focus_mode  = 0;
    int  prev_focus_mode = 0;
    bool vf_dd_edit      = false;

    Camera3D camera = {};
    camera.up         = { 0, 1, 0 };
    camera.fovy       = 45.f;
    camera.projection = CAMERA_PERSPECTIVE;

    AtmosphericConditions atmo = isa_conditions(0.0);

    const int PANEL_W = 340;

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    while (!WindowShouldClose())
    {
        const int   W  = GetScreenWidth();
        const int   H  = GetScreenHeight();
        const float dt = GetFrameTime();
        const bool  mouse_in_3d = (GetMouseX() >= PANEL_W);

        // -------------------------------------------------------------------
        // Update muzzle speed default when munition selection changes
        // -------------------------------------------------------------------
        if (mun_idx != last_mun_for_speed) {
            muzzle_speed = (float)lib.get(mun_names[(size_t)mun_idx]).muzzle_velocity_ms;
            last_mun_for_speed = mun_idx;
        }

        // -------------------------------------------------------------------
        // Moving target: capture origin on toggle-on
        // -------------------------------------------------------------------
        if (target_moving && !prev_target_moving) {
            target_origin_x    = tx;
            target_origin_y    = ty;
            target_travel_prog = 0.f;
            target_travel_fwd  = true;
        }
        prev_target_moving = target_moving;

        // -------------------------------------------------------------------
        // Keyboard controls (disabled while dropdown is open)
        // -------------------------------------------------------------------
        if (!dd_edit && !vf_dd_edit) {
            const float spd = 80.f * dt;   // m/s movement speed
            // Target movement (disabled when moving target is active)
            if (!target_moving) {
                if (IsKeyDown(KEY_W)) ty += spd;
                if (IsKeyDown(KEY_S)) ty -= spd;
                if (IsKeyDown(KEY_D)) tx += spd;
                if (IsKeyDown(KEY_A)) tx -= spd;
                if (IsKeyDown(KEY_E)) tz += spd * 0.3f;
                if (IsKeyDown(KEY_Q)) tz -= spd * 0.3f;
            }
            // Launcher movement
            if (IsKeyDown(KEY_UP))        ly += spd;
            if (IsKeyDown(KEY_DOWN))      ly -= spd;
            if (IsKeyDown(KEY_RIGHT))     lx += spd;
            if (IsKeyDown(KEY_LEFT))      lx -= spd;
            if (IsKeyDown(KEY_PAGE_UP))   lz += spd * 0.3f;
            if (IsKeyDown(KEY_PAGE_DOWN)) lz -= spd * 0.3f;
        }

        // Clamp altitudes to >= 0
        tz = std::max(tz, 0.f);
        lz = std::max(lz, 0.f);

        // -------------------------------------------------------------------
        // Moving target: update position each frame (ping-pong)
        // -------------------------------------------------------------------
        if (target_moving && target_travel_dist > 0.f) {
            if (target_travel_prog > target_travel_dist)
                target_travel_prog = target_travel_dist;

            float step = target_speed * dt;
            if (target_travel_fwd) {
                target_travel_prog += step;
                if (target_travel_prog >= target_travel_dist) {
                    target_travel_prog = target_travel_dist;
                    target_travel_fwd  = false;
                }
            } else {
                target_travel_prog -= step;
                if (target_travel_prog <= 0.f) {
                    target_travel_prog = 0.f;
                    target_travel_fwd  = true;
                }
            }

            float yaw_rad = target_yaw * (float)kDegToRad;
            tx = target_origin_x + target_travel_prog * sinf(yaw_rad);
            ty = target_origin_y + target_travel_prog * cosf(yaw_rad);
        }

        // -------------------------------------------------------------------
        // Detect input changes → mark dirty
        // -------------------------------------------------------------------
        if (lx != p_lx || ly != p_ly || lz != p_lz ||
            tx != p_tx || ty != p_ty || tz != p_tz ||
            mun_idx != p_mi || muzzle_speed != p_mv)
        {
            dirty = true;
            p_lx = lx; p_ly = ly; p_lz = lz;
            p_tx = tx; p_ty = ty; p_tz = tz;
            p_mi = mun_idx;
            p_mv = muzzle_speed;
        }

        // -------------------------------------------------------------------
        // Kick off / collect async fire solution
        // -------------------------------------------------------------------
        if (!computing && dirty) {
            dirty     = false;
            computing = true;

            SolveParams sp;
            sp.launcher_pos = { lx, ly, lz };
            sp.target_pos   = { tx, ty, tz };
            sp.munition     = lib.get(mun_names[(size_t)mun_idx]);
            sp.atmo         = atmo;
            sp.muzzle_speed = (double)muzzle_speed;

            pending = std::async(std::launch::async, solve_async, sp);
        }

        if (computing && pending.valid()) {
            if (pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                current   = pending.get();
                computing = false;
            }
        }

        // -------------------------------------------------------------------
        // Camera orbit (right-mouse drag) + zoom (scroll wheel)
        // -------------------------------------------------------------------
        if (mouse_in_3d) {
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d = GetMouseDelta();
                cam_az += d.x * 0.4f;
                cam_el -= d.y * 0.4f;
                cam_el  = Clamp(cam_el, 3.f, 89.f);
            }
            // Pan (middle-mouse drag) — only in Free mode
            if (cam_focus_mode == 0 && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
                Vector2 d = GetMouseDelta();
                float scale = cam_dist * 0.002f; // pan speed scales with zoom
                float az_r_cam = cam_az * DEG2RAD;
                // Move in the camera's local horizontal plane
                pan_x -= (d.x * cosf(az_r_cam) + d.y * sinf(az_r_cam) * sinf(cam_el * DEG2RAD)) * scale;
                pan_z += (d.x * sinf(az_r_cam) - d.y * cosf(az_r_cam) * sinf(cam_el * DEG2RAD)) * scale;
            }
            cam_dist -= GetMouseWheelMove() * cam_dist * 0.08f;
            cam_dist  = Clamp(cam_dist, 5.f, 8000.f);
        }

        // Reset pan when switching to a snap mode
        if (cam_focus_mode != prev_focus_mode) {
            if (cam_focus_mode != 0) { pan_x = 0.f; pan_z = 0.f; }
            prev_focus_mode = cam_focus_mode;
        }

        // Orbit focus based on view mode
        Vector3 rl_l   = to_rl({ lx, ly, lz });
        Vector3 rl_t   = to_rl({ tx, ty, tz });
        Vector3 focus;
        if (cam_focus_mode == 1)      focus = rl_l;                                    // Launcher
        else if (cam_focus_mode == 2) focus = rl_t;                                    // Target
        else                          focus = Vector3Scale(Vector3Add(rl_l, rl_t), 0.5f); // Free
        focus.x += pan_x;
        focus.z += pan_z;

        const float el_r = cam_el * DEG2RAD;
        const float az_r = cam_az * DEG2RAD;
        camera.target   = focus;
        camera.position = {
            focus.x + cam_dist * cosf(el_r) * sinf(az_r),
            focus.y + cam_dist * sinf(el_r),
            focus.z + cam_dist * cosf(el_r) * cosf(az_r)
        };

        // -------------------------------------------------------------------
        // Azimuth/elevation for launcher drawing
        // Use computed solution when valid; fall back to geometric azimuth
        // -------------------------------------------------------------------
        float disp_az = 0.f, disp_el = 0.f;
        if (current.valid) {
            disp_az = (float)current.azimuth_deg;
            disp_el = (float)current.elevation_deg;
        } else {
            const double dx = tx - lx, dy = ty - ly;
            disp_az = (float)(std::atan2(dx, dy) * kRadToDeg);
        }

        // ===================================================================
        // RENDER
        // ===================================================================
        BeginDrawing();

        // ---- 3D viewport (right of panel) ---------------------------------
        ClearBackground({ 20, 23, 34, 255 });

        BeginScissorMode(PANEL_W, 0, W - PANEL_W, H);
        BeginMode3D(camera);

        DrawGrid(800, 10.0f);   // 8 km × 8 km, 10 m cells

        // Axis arrows at world origin
        DrawLine3D({ 0, 0.1f,  0  }, { 25, 0.1f,   0  }, RED);    // +x = East
        DrawLine3D({ 0, 0.1f,  0  }, {  0, 0.1f, -25  }, GREEN);  // -z = North
        DrawLine3D({ 0, 0,     0  }, {  0, 25,     0  }, BLUE);   // +y = Up

        draw_launcher(rl_l, disp_az, disp_el);
        draw_target(rl_t);

        // Moving target: draw travel path
        if (target_moving && target_travel_dist > 0.f) {
            float yaw_rad = target_yaw * (float)kDegToRad;
            Vec3 path_start = { target_origin_x, target_origin_y, 0.0 };
            Vec3 path_end   = {
                target_origin_x + target_travel_dist * sinf(yaw_rad),
                target_origin_y + target_travel_dist * cosf(yaw_rad),
                0.0
            };
            Vector3 rl_ps = to_rl(path_start);
            Vector3 rl_pe = to_rl(path_end);
            rl_ps.y = 0.05f;
            rl_pe.y = 0.05f;
            DrawLine3D(rl_ps, rl_pe, { 255, 100, 100, 200 });
            DrawSphere({ rl_ps.x, 0.1f, rl_ps.z }, 0.4f, { 200, 200, 50, 200 });
            DrawSphere({ rl_pe.x, 0.1f, rl_pe.z }, 0.4f, { 200, 200, 50, 200 });
        }

        if (current.valid && !current.traj.empty()) {
            draw_trajectory(current.traj);
            // Impact marker at trajectory end
            DrawSphere(current.traj.back(), 0.55f, { 255, 200, 0, 255 });
        }

        EndMode3D();

        // Axis labels (screen-space, projected from 3D points)
        Vector2 scr_e = GetWorldToScreen({ 25, 0.1f,   0  }, camera);
        Vector2 scr_n = GetWorldToScreen({  0, 0.1f, -25  }, camera);
        Vector2 scr_u = GetWorldToScreen({  0, 25,     0  }, camera);
        DrawText("E",  (int)scr_e.x + 3, (int)scr_e.y - 8,  15, RED);
        DrawText("N",  (int)scr_n.x + 3, (int)scr_n.y - 8,  15, GREEN);
        DrawText("Up", (int)scr_u.x + 3, (int)scr_u.y - 8,  15, BLUE);

        if (computing)
            DrawText("[ computing... ]", PANEL_W + 12, 12, 14, YELLOW);

        EndScissorMode();

        // ---- GUI panel (left side) ----------------------------------------
        DrawRectangle(0, 0, PANEL_W, H, { 40, 43, 54, 255 });
        DrawRectangle(PANEL_W - 2, 0, 2, H, { 65, 70, 95, 255 }); // panel border

        const int  mx = 12;              // horizontal margin
        const int  cw = PANEL_W - mx*2; // usable control width
        const int  rh = 26;             // row height
        const Color sec_col  = { 150, 155, 180, 255 }; // section label colour
        const Color val_col  = WHITE;
        const Color ctrl_col = LIGHTGRAY;
        int y = 12;

        DrawText("Ballistics Renderer", mx, y, 18, WHITE); y += 34;

        // ---- Munition -------------------------------------------------------
        DrawText("MUNITION", mx, y, 12, sec_col); y += 16;
        const int mun_dd_y = y; // save Y for deferred dropdown draw
        y += rh + 6;

        // Muzzle speed slider
        DrawText("Muzzle speed (m/s)", mx, y, 12, sec_col); y += 16;
        GuiSliderBar({ (float)mx, (float)y, (float)(cw - 68), (float)rh },
                     nullptr, nullptr, &muzzle_speed, 100.f, 1500.f);
        DrawText(TextFormat("%.0f", muzzle_speed), mx + cw - 62, y + 6, 14, val_col);
        y += rh + 14;

        // ---- Launcher -------------------------------------------------------
        DrawText("LAUNCHER  (x=East, y=North, z=Alt  metres)", mx, y, 12, sec_col); y += 17;

        DrawText("X(E)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &lx, -2000.f, 2000.f);
        DrawText(TextFormat("%+.0f", lx), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Y(N)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &ly, -2000.f, 2000.f);
        DrawText(TextFormat("%+.0f", ly), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Alt", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &lz, 0.f, 200.f);
        DrawText(TextFormat("%.1f", lz), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 14;

        // ---- Target ---------------------------------------------------------
        DrawText("TARGET  (x=East, y=North, z=Alt  metres)", mx, y, 12, sec_col); y += 17;

        if (target_moving) GuiLock();

        DrawText("X(E)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &tx, -2000.f, 2000.f);
        DrawText(TextFormat("%+.0f", tx), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Y(N)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &ty, -2000.f, 2000.f);
        DrawText(TextFormat("%+.0f", ty), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Alt", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({ (float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh },
                     nullptr, nullptr, &tz, 0.f, 500.f);
        DrawText(TextFormat("%.1f", tz), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 10;

        if (target_moving) GuiUnlock();

        // ---- Moving Target --------------------------------------------------
        DrawText("MOVING TARGET", mx, y, 12, sec_col); y += 17;

        GuiCheckBox({ (float)mx, (float)y, 20.f, 20.f }, "Enable", &target_moving);
        y += rh + 3;

        DrawText("Speed (m/s)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({ (float)mx, (float)(y + 18), (float)(cw - 68), (float)rh },
                     nullptr, nullptr, &target_speed, 0.5f, 200.f);
        DrawText(TextFormat("%.1f", target_speed), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 22;

        DrawText("Heading (deg)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({ (float)mx, (float)(y + 18), (float)(cw - 68), (float)rh },
                     nullptr, nullptr, &target_yaw, 0.f, 360.f);
        DrawText(TextFormat("%.0f", target_yaw), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 22;

        DrawText("Distance (m)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({ (float)mx, (float)(y + 18), (float)(cw - 68), (float)rh },
                     nullptr, nullptr, &target_travel_dist, 10.f, 2000.f);
        DrawText(TextFormat("%.0f", target_travel_dist), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 26;

        // ---- Fire Solution --------------------------------------------------
        DrawText("FIRE SOLUTION", mx, y, 12, sec_col); y += 18;

        if (current.valid) {
            const Color fc = { 75, 215, 100, 255 };
            DrawText(TextFormat("Azimuth   : %7.2f deg", current.azimuth_deg),   mx, y, 14, fc); y += 20;
            DrawText(TextFormat("Elevation : %7.2f deg", current.elevation_deg), mx, y, 14, fc); y += 20;
            DrawText(TextFormat("Range     : %7.0f m",   current.range_m),       mx, y, 14, fc); y += 20;
            DrawText(TextFormat("Flight T  : %7.3f s",   current.flight_time_s), mx, y, 14, fc); y += 20;
        } else if (computing) {
            DrawText("  Computing...", mx, y, 14, YELLOW); y += 20;
        } else {
            const Color err_col = { 220, 80, 80, 255 };
            DrawText("  No solution", mx, y, 14, err_col); y += 18;
            DrawText(TextFormat("  Range: %.0f m  Max: %.0f m",
                     current.range_m, current.max_range_m), mx, y, 12, err_col); y += 16;
            DrawText(TextFormat("  Alt diff: %+.1f m (launcher - target)",
                     current.alt_diff_m), mx, y, 12, err_col); y += 16;
        }

        // ---- View Focus -----------------------------------------------------
        DrawText("VIEW FOCUS", mx, y, 12, sec_col); y += 16;
        const int vf_dd_y = y; // save Y for deferred dropdown draw
        y += rh + 14;

        // ---- Keyboard help (anchored to bottom of panel) --------------------
        int hy = H - 210;
        DrawRectangle(mx - 4, hy - 8, cw + 8, 214, { 28, 30, 40, 200 });
        DrawText("CONTROLS", mx, hy, 12, sec_col); hy += 18;
        const Color hc = { 175, 178, 200, 255 };
        DrawText("W / S / A / D      target  N / S / E / W", mx, hy, 12, hc); hy += 16;
        DrawText("Q / E              target  altitude",       mx, hy, 12, hc); hy += 16;
        DrawText("  (disabled while moving target is active)", mx, hy, 11, { 140, 140, 160, 200 }); hy += 16;
        DrawText("Arrow keys         launcher N / S / E / W", mx, hy, 12, hc); hy += 16;
        DrawText("Page Up / Down     launcher altitude",       mx, hy, 12, hc); hy += 16;
        DrawText("Right-mouse drag   orbit camera",            mx, hy, 12, hc); hy += 16;
        DrawText("Middle-mouse drag  pan camera",              mx, hy, 12, hc); hy += 16;
        DrawText("Scroll wheel       zoom",                    mx, hy, 12, hc);

        // ---- Deferred dropdown draws (on top of all other controls) ---------
        if (GuiDropdownBox({ (float)mx, (float)vf_dd_y, (float)cw, (float)rh },
                            "Free;Launcher;Target", &cam_focus_mode, vf_dd_edit))
            vf_dd_edit = !vf_dd_edit;

        if (GuiDropdownBox({ (float)mx, (float)mun_dd_y, (float)cw, (float)rh },
                            dd_str.c_str(), &mun_idx, dd_edit))
            dd_edit = !dd_edit;

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
