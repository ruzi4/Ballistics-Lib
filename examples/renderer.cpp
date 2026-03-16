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
#include "rlgl.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <ballistics/ballistics.hpp>
#include <ballistics/math/math_constants.hpp>
#include <nlohmann/json.hpp>

using namespace ballistics;

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------
static inline Vector3 to_rl(const Vec3& v) {
    return {(float)v.x, (float)v.z, -(float)v.y};
}

// ---------------------------------------------------------------------------
// begin_mode3d_ex — BeginMode3D replacement with configurable clip planes
// ---------------------------------------------------------------------------
// raylib's BeginMode3D() uses compile-time constants RL_CULL_DISTANCE_NEAR
// (0.01 m) and RL_CULL_DISTANCE_FAR (1000 m) baked into the library binary,
// making it impossible to extend the far plane via macros at the call site.
// This function replicates BeginMode3D's rlgl calls with caller-supplied
// near/far values and uses the actual scissored viewport aspect ratio so the
// 3D view is undistorted even though it shares the window with the GUI panel.
// Call EndMode3D() normally after use — it only pops the matrix stack.
static void begin_mode3d_ex(Camera3D cam, float near_z, float far_z, int vp_w, int vp_h) {
    rlDrawRenderBatchActive(); // flush any pending 2D batch

    rlMatrixMode(RL_PROJECTION);
    rlPushMatrix();
    rlLoadIdentity();

    const float  aspect = (vp_h > 0) ? (float)vp_w / (float)vp_h : 1.f;
    const double top    = (double)near_z * tan(cam.fovy * 0.5 * DEG2RAD);
    const double right  = top * aspect;
    rlFrustum(-right, right, -top, top, (double)near_z, (double)far_z);

    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();

    Matrix view = MatrixLookAt(cam.position, cam.target, cam.up);
    rlMultMatrixf(MatrixToFloat(view));

    rlEnableDepthTest();
}

// ---------------------------------------------------------------------------
// Trajectory conversion helper: ballistics Vec3 → raylib Vector3
// ---------------------------------------------------------------------------
static std::vector<Vector3> traj_to_rl(const std::vector<Vec3>& pts) {
    std::vector<Vector3> out;
    out.reserve(pts.size());
    for (const auto& p : pts)
        out.push_back(to_rl(p));
    return out;
}

// ---------------------------------------------------------------------------
// Draw launcher: box body + rotating barrel cylinder + muzzle direction line
// scale > 1 keeps the model visible when the camera is far away.
// ---------------------------------------------------------------------------
static void draw_launcher(Vector3 pos, float az_deg, float el_deg, float scale = 1.f) {
    // Body
    DrawCube(pos, 3.0f * scale, 1.2f * scale, 2.0f * scale, {55, 115, 55, 255});
    DrawCubeWires(pos, 3.0f * scale, 1.2f * scale, 2.0f * scale, {0, 70, 0, 255});

    // Firing direction in raylib coords:
    //   az=0   → -z (North),  az=90 → +x (East)
    //   el > 0 → upward (+y)
    const float az_r = az_deg * DEG2RAD;
    const float el_r = el_deg * DEG2RAD;
    Vector3     dir  = {
        sinf(az_r) * cosf(el_r), // East  (+x)
        sinf(el_r),              // Up    (+y)
        -cosf(az_r) * cosf(el_r) // South (+z); negated → North
    };

    // Barrel (cylinder from top-center of box outward)
    Vector3 barrel_base = {pos.x, pos.y + 0.6f * scale, pos.z};
    Vector3 barrel_tip  = Vector3Add(barrel_base, Vector3Scale(dir, 3.5f * scale));
    DrawCylinderEx(barrel_base, barrel_tip, 0.14f * scale, 0.11f * scale, 10, {100, 100, 100, 255});
    DrawCylinderWiresEx(
        barrel_base, barrel_tip, 0.14f * scale, 0.11f * scale, 10, {55, 55, 55, 255});

    // Muzzle direction line (red ray extending from barrel tip)
    Vector3 muzzle_end = Vector3Add(barrel_tip, Vector3Scale(dir, 5.0f * scale));
    DrawLine3D(barrel_tip, muzzle_end, RED);
}

// ---------------------------------------------------------------------------
// Draw target: sphere + vertical stake + ground crosshair
// ---------------------------------------------------------------------------
static void draw_target(Vector3 pos, float scale = 1.f) {
    DrawSphere(pos, 0.6f * scale, {220, 50, 50, 255});
    DrawSphereWires(pos, 0.6f * scale, 8, 8, {150, 20, 20, 255});

    // Vertical stake to ground plane
    Vector3 ground = {pos.x, 0.02f, pos.z};
    DrawLine3D(ground, pos, {200, 80, 80, 180});

    // Crosshair on ground
    const float arm = 2.0f * scale;
    const float gy  = 0.03f;
    DrawLine3D({pos.x - arm, gy, pos.z}, {pos.x + arm, gy, pos.z}, {220, 80, 80, 255});
    DrawLine3D({pos.x, gy, pos.z - arm}, {pos.x, gy, pos.z + arm}, {220, 80, 80, 255});
}

// ---------------------------------------------------------------------------
// Draw trajectory arc + apex marker
// ---------------------------------------------------------------------------
static void draw_trajectory(const std::vector<Vector3>& pts, float scale = 1.f) {
    if (pts.size() < 2)
        return;

    // Find apex (highest y = Up in raylib)
    size_t apex = 0;
    for (size_t i = 1; i < pts.size(); ++i)
        if (pts[i].y > pts[apex].y)
            apex = i;

    for (size_t i = 1; i < pts.size(); ++i)
        DrawLine3D(pts[i - 1], pts[i], {255, 165, 0, 255});

    // Apex marker
    if (apex > 0 && apex < pts.size() - 1)
        DrawSphere(pts[apex], 0.4f * scale, YELLOW);
}

// ---------------------------------------------------------------------------
int main() {
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
        if (i)
            dd_str += ';';
        dd_str += mun_names[i];
    }

    // -----------------------------------------------------------------------
    // Scene state  (metres, ballistics coords: x=East, y=North, z=Up)
    // -----------------------------------------------------------------------
    float lx = 0.f, ly = 0.f, lz = 1.5f;    // launcher position
    float tx = 500.f, ty = 300.f, tz = 0.f; // target position

    // Moving target state
    bool  target_moving      = false;
    float target_speed       = 20.f;  // m/s
    float target_yaw         = 0.f;   // degrees, 0=North, 90=East
    float target_travel_dist = 200.f; // metres
    float target_travel_prog = 0.f;   // current progress (0..dist), ping-pong
    bool  target_travel_fwd  = true;  // ping-pong direction
    float target_origin_x    = tx;    // captured on toggle-on
    float target_origin_y    = ty;
    bool  prev_target_moving = false;

    // -----------------------------------------------------------------------
    // Load launcher configuration (slew rates) from JSON
    // -----------------------------------------------------------------------
    LauncherSlew slew_cfg; // defaults: 25 deg/s yaw and pitch
    {
        std::ifstream f("data/launcher_config.json");
        if (f.is_open()) {
            try {
                auto j = nlohmann::json::parse(f);
                if (j.contains("slew_rate_yaw_deg_per_s") &&
                    j["slew_rate_yaw_deg_per_s"].is_number())
                    slew_cfg.yaw_deg_per_s = j["slew_rate_yaw_deg_per_s"].get<double>();
                if (j.contains("slew_rate_pitch_deg_per_s") &&
                    j["slew_rate_pitch_deg_per_s"].is_number())
                    slew_cfg.pitch_deg_per_s = j["slew_rate_pitch_deg_per_s"].get<double>();
            } catch (...) { /* malformed JSON — keep defaults */
            }
        }
    }

    int   mun_idx      = 1; // 5.56×45 M855 as default
    float muzzle_speed = (float)lib.get(mun_names[(size_t)mun_idx]).muzzle_velocity_ms;
    bool  dd_edit      = false;

    // Auto-update muzzle speed when munition changes
    int last_mun_for_speed = mun_idx;

    // -----------------------------------------------------------------------
    // Physical launcher orientation — smoothly slewed toward the fire solution
    // -----------------------------------------------------------------------
    // Initialised to point at the default target so the first frame looks sensible.
    float phys_az = (float)(std::atan2((double)(tx - lx), (double)(ty - ly)) * kRadToDeg);
    float phys_el = 0.f;

    // -----------------------------------------------------------------------
    // Async fire-solution state (uses library AsyncSolver)
    // -----------------------------------------------------------------------
    AsyncSolver          solver;
    bool                 dirty = true;
    std::vector<Vector3> traj_rl; // trajectory cache in raylib coords

    // Previous values — used to detect changes and trigger recompute
    float p_lx = lx - 1.f, p_ly = ly, p_lz = lz;
    float p_tx = tx, p_ty = ty, p_tz = tz;
    int   p_mi = mun_idx - 1;
    float p_mv = muzzle_speed - 1.f;
    bool  p_tm = !target_moving; // force dirty on first frame

    // -----------------------------------------------------------------------
    // Camera orbit state
    // -----------------------------------------------------------------------
    float cam_dist = 280.f;
    float cam_az   = 30.f; // horizontal orbit angle (degrees)
    float cam_el   = 35.f; // vertical elevation (degrees)
    float pan_x    = 0.f;  // pan offset in raylib x (East)
    float pan_z    = 0.f;  // pan offset in raylib z (South)

    // View focus: 0 = Free (midpoint + pan), 1 = Launcher, 2 = Target
    int  cam_focus_mode  = 0;
    int  prev_focus_mode = 0;
    bool vf_dd_edit      = false;

    // Legend overlay visibility
    bool     show_scene_legend = true;
    bool     show_help_legend  = false;

    Camera3D camera   = {};
    camera.up         = {0, 1, 0};
    camera.fovy       = 45.f;
    camera.projection = CAMERA_PERSPECTIVE;

    AtmosphericConditions atmo = isa_conditions(0.0);

    const int             PANEL_W = 340;

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    while (!WindowShouldClose()) {
        const int   W           = GetScreenWidth();
        const int   H           = GetScreenHeight();
        const float dt          = GetFrameTime();
        const bool  mouse_in_3d = (GetMouseX() >= PANEL_W);

        // -------------------------------------------------------------------
        // Update muzzle speed default when munition selection changes
        // -------------------------------------------------------------------
        if (mun_idx != last_mun_for_speed) {
            muzzle_speed       = (float)lib.get(mun_names[(size_t)mun_idx]).muzzle_velocity_ms;
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
            // Movement speed scales with camera distance so the scene is still
            // navigable whether the camera is 50 m or 10 km away.
            const float spd = std::max(80.f, cam_dist * 0.15f) * dt;
            // Target movement (disabled when moving target is active)
            if (!target_moving) {
                if (IsKeyDown(KEY_W))
                    ty += spd;
                if (IsKeyDown(KEY_S))
                    ty -= spd;
                if (IsKeyDown(KEY_D))
                    tx += spd;
                if (IsKeyDown(KEY_A))
                    tx -= spd;
                if (IsKeyDown(KEY_E))
                    tz += spd * 0.3f;
                if (IsKeyDown(KEY_Q))
                    tz -= spd * 0.3f;
            }
            // Launcher movement
            if (IsKeyDown(KEY_UP))
                ly += spd;
            if (IsKeyDown(KEY_DOWN))
                ly -= spd;
            if (IsKeyDown(KEY_RIGHT))
                lx += spd;
            if (IsKeyDown(KEY_LEFT))
                lx -= spd;
            if (IsKeyDown(KEY_PAGE_UP))
                lz += spd * 0.3f;
            if (IsKeyDown(KEY_PAGE_DOWN))
                lz -= spd * 0.3f;
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
            tx            = target_origin_x + target_travel_prog * sinf(yaw_rad);
            ty            = target_origin_y + target_travel_prog * cosf(yaw_rad);
        }

        // -------------------------------------------------------------------
        // Detect input changes → mark dirty
        // -------------------------------------------------------------------
        if (lx != p_lx || ly != p_ly || lz != p_lz || tx != p_tx || ty != p_ty || tz != p_tz ||
            mun_idx != p_mi || muzzle_speed != p_mv || target_moving != p_tm) {
            dirty = true;
            p_lx  = lx;
            p_ly  = ly;
            p_lz  = lz;
            p_tx  = tx;
            p_ty  = ty;
            p_tz  = tz;
            p_mi  = mun_idx;
            p_mv  = muzzle_speed;
            p_tm  = target_moving;
        }

        // -------------------------------------------------------------------
        // Kick off / collect async fire solution (via library AsyncSolver)
        // -------------------------------------------------------------------
        if (dirty) {
            dirty = false;

            SolveParams sp;
            sp.launcher_pos  = {lx, ly, lz};
            sp.target_pos    = {tx, ty, tz};
            sp.target_moving = target_moving;
            if (target_moving) {
                const float yaw_r  = target_yaw * (float)kDegToRad;
                const float dir    = target_travel_fwd ? 1.f : -1.f;
                sp.target_velocity = {(double)(dir * target_speed * std::sin(yaw_r)),
                                      (double)(dir * target_speed * std::cos(yaw_r)),
                                      0.0};
            } else {
                sp.target_velocity = {0.0, 0.0, 0.0};
            }
            sp.current_azimuth_deg   = (double)phys_az;
            sp.current_elevation_deg = (double)phys_el;
            sp.slew                  = slew_cfg;
            sp.munition              = lib.get(mun_names[(size_t)mun_idx]);
            sp.atmosphere            = atmo;
            sp.muzzle_speed_ms       = (double)muzzle_speed;

            solver.request(sp);
        }

        // Update raylib trajectory cache when a new result arrives
        if (solver.poll())
            traj_rl = traj_to_rl(solver.result().trajectory);

        const SolveResult& current = solver.result();

        // -------------------------------------------------------------------
        // Slew physical launcher toward commanded fire-solution angles
        // -------------------------------------------------------------------
        {
            // Commanded angles: use fire solution when valid, otherwise point
            // at the target geometrically with zero elevation.
            float cmd_az, cmd_el;
            if (current.valid) {
                cmd_az = (float)current.azimuth_deg;
                cmd_el = (float)current.elevation_deg;
            } else {
                const double dx = tx - lx, dy = ty - ly;
                cmd_az = (float)(std::atan2(dx, dy) * kRadToDeg);
                cmd_el = 0.f;
            }

            // Maximum angular change this frame
            const float max_daz = (float)slew_cfg.yaw_deg_per_s * dt;
            const float max_del = (float)slew_cfg.pitch_deg_per_s * dt;

            // Azimuth: shortest-path difference wrapped to [-180, 180]
            float daz = std::fmod(cmd_az - phys_az + 540.f, 360.f) - 180.f;
            phys_az =
                std::fmod(phys_az + std::fmax(-max_daz, std::fmin(max_daz, daz)) + 360.f, 360.f);

            // Elevation: simple clamped step
            float del = cmd_el - phys_el;
            phys_el += std::fmax(-max_del, std::fmin(max_del, del));
        }

        // -------------------------------------------------------------------
        // Camera orbit (right-mouse drag) + zoom (scroll wheel)
        // -------------------------------------------------------------------
        if (mouse_in_3d) {
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d = GetMouseDelta();
                cam_az += d.x * 0.4f;
                cam_el -= d.y * 0.4f;
                cam_el = Clamp(cam_el, 3.f, 89.f);
            }
            // Pan (middle-mouse drag) — only in Free mode
            if (cam_focus_mode == 0 && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
                Vector2 d        = GetMouseDelta();
                float   scale    = cam_dist * 0.002f; // pan speed scales with zoom
                float   az_r_cam = cam_az * DEG2RAD;
                // Move in the camera's local horizontal plane
                pan_x -=
                    (d.x * cosf(az_r_cam) + d.y * sinf(az_r_cam) * sinf(cam_el * DEG2RAD)) * scale;
                pan_z +=
                    (d.x * sinf(az_r_cam) - d.y * cosf(az_r_cam) * sinf(cam_el * DEG2RAD)) * scale;
            }
            cam_dist -= GetMouseWheelMove() * cam_dist * 0.08f;
            cam_dist = Clamp(cam_dist, 5.f, 20000.f);
        }

        // Reset pan when switching to a snap mode
        if (cam_focus_mode != prev_focus_mode) {
            if (cam_focus_mode != 0) {
                pan_x = 0.f;
                pan_z = 0.f;
            }
            prev_focus_mode = cam_focus_mode;
        }

        // Orbit focus based on view mode
        Vector3 rl_l = to_rl({lx, ly, lz});
        Vector3 rl_t = to_rl({tx, ty, tz});
        Vector3 focus;
        if (cam_focus_mode == 1)
            focus = rl_l; // Launcher
        else if (cam_focus_mode == 2)
            focus = rl_t; // Target
        else
            focus = Vector3Scale(Vector3Add(rl_l, rl_t), 0.5f); // Free
        focus.x += pan_x;
        focus.z += pan_z;

        const float el_r = cam_el * DEG2RAD;
        const float az_r = cam_az * DEG2RAD;
        camera.target    = focus;
        camera.position  = {focus.x + cam_dist * cosf(el_r) * sinf(az_r),
                            focus.y + cam_dist * sinf(el_r),
                            focus.z + cam_dist * cosf(el_r) * cosf(az_r)};

        // The launcher model always follows the physical (slewed) angles.

        // ===================================================================
        // RENDER
        // ===================================================================
        BeginDrawing();

        // ---- 3D viewport (right of panel) ---------------------------------
        ClearBackground({20, 23, 34, 255});

        BeginScissorMode(PANEL_W, 0, W - PANEL_W, H);
        // Use extended far-clip (50 km) so objects at 5 km range are not culled.
        begin_mode3d_ex(camera, 0.5f, 50000.f, W - PANEL_W, H);

        // Adaptive grid: cell size and count scale with camera distance so the
        // grid always covers the scene without becoming too dense or too sparse.
        {
            float cell = 10.f;
            if (cam_dist > 5000.f)
                cell = 500.f;
            else if (cam_dist > 2000.f)
                cell = 200.f;
            else if (cam_dist > 800.f)
                cell = 50.f;
            else if (cam_dist > 300.f)
                cell = 20.f;
            // Always draw at least 500 cells so the grid extends far enough.
            int slices = (int)std::ceil(cam_dist * 2.f / cell);
            slices     = std::max(slices, 200);
            DrawGrid(slices, cell);
        }

        // Axis arrows — length scales with camera distance so they stay visible.
        const float ax_len = std::max(25.f, cam_dist * 0.05f);
        DrawLine3D({0, 0.1f, 0}, {ax_len, 0.1f, 0}, RED);    // +x = East
        DrawLine3D({0, 0.1f, 0}, {0, 0.1f, -ax_len}, GREEN); // -z = North
        DrawLine3D({0, 0, 0}, {0, ax_len, 0}, BLUE);         // +y = Up

        // Per-object scale: keep markers visible proportional to camera distance.
        const float launcher_scale = std::max(1.f, Vector3Distance(camera.position, rl_l) / 200.f);
        const float target_scale   = std::max(1.f, Vector3Distance(camera.position, rl_t) / 200.f);
        const float traj_scale     = std::max(1.f, cam_dist / 200.f);

        draw_launcher(rl_l, phys_az, phys_el, launcher_scale);
        draw_target(rl_t, target_scale);

        // Moving target: draw travel path
        if (target_moving && target_travel_dist > 0.f) {
            float   yaw_rad    = target_yaw * (float)kDegToRad;
            Vec3    path_start = {target_origin_x, target_origin_y, (double)tz};
            Vec3    path_end   = {target_origin_x + target_travel_dist * sinf(yaw_rad),
                                  target_origin_y + target_travel_dist * cosf(yaw_rad),
                                  (double)tz};
            Vector3 rl_ps      = to_rl(path_start);
            Vector3 rl_pe      = to_rl(path_end);
            DrawLine3D(rl_ps, rl_pe, {255, 100, 100, 200});
            DrawSphere({rl_ps.x, rl_t.y, rl_ps.z}, 0.4f * traj_scale, {200, 200, 50, 200});
            DrawSphere({rl_pe.x, rl_t.y, rl_pe.z}, 0.4f * traj_scale, {200, 200, 50, 200});
        }

        if (current.valid && !traj_rl.empty()) {
            draw_trajectory(traj_rl, traj_scale);
            // Impact marker at trajectory end
            DrawSphere(traj_rl.back(), 0.55f * traj_scale, {255, 200, 0, 255});
        }

        // Moving-target intercept: draw predicted intercept point + lead vector
        if (current.valid && current.has_intercept) {
            Vector3     rl_ipt    = to_rl(current.intercept_point);
            const float ipt_scale = std::max(1.f, Vector3Distance(camera.position, rl_ipt) / 200.f);

            // Cyan sphere at predicted intercept position
            DrawSphere(rl_ipt, 0.8f * ipt_scale, {0, 220, 220, 220});
            DrawSphereWires(rl_ipt, 0.8f * ipt_scale, 8, 8, {0, 160, 160, 255});

            // Vertical dashed stake from ground to intercept sphere
            Vector3 ipt_ground = {rl_ipt.x, 0.04f, rl_ipt.z};
            DrawLine3D(ipt_ground, rl_ipt, {0, 180, 180, 160});

            // Lead vector: line from current target to intercept point at target altitude
            DrawLine3D(
                {rl_t.x, rl_t.y, rl_t.z}, {ipt_ground.x, rl_t.y, ipt_ground.z}, {0, 255, 255, 200});
        }

        EndMode3D();

        // Axis labels (screen-space, projected from 3D points at arrow tips)
        Vector2 scr_e = GetWorldToScreen({ax_len, 0.1f, 0}, camera);
        Vector2 scr_n = GetWorldToScreen({0, 0.1f, -ax_len}, camera);
        Vector2 scr_u = GetWorldToScreen({0, ax_len, 0}, camera);
        DrawText("E", (int)scr_e.x + 3, (int)scr_e.y - 8, 15, RED);
        DrawText("N", (int)scr_n.x + 3, (int)scr_n.y - 8, 15, GREEN);
        DrawText("Up", (int)scr_u.x + 3, (int)scr_u.y - 8, 15, BLUE);

        if (solver.computing())
            DrawText("[ computing... ]", PANEL_W + 12, 12, 14, YELLOW);

        EndScissorMode();

        // ---- Legend toggle buttons (top-right corner of 3D viewport) --------
        GuiToggle({(float)(W - 138), 8.f, 62.f, 22.f}, "Scene", &show_scene_legend);
        GuiToggle({(float)(W - 72), 8.f, 62.f, 22.f}, "Help", &show_help_legend);

        // ---- Scene legend overlay -------------------------------------------
        if (show_scene_legend) {
            const int slx  = W - 215;
            const int sly0 = 36;
            const int slw  = 207;
            const int slrh = 18; // row height
            const int slp  = 8;  // left padding inside box
            // 1 title row + 9 data rows; top+bottom padding = 8px each
            const int slh = 8 + 10 * slrh + 8;
            int       sly = sly0 + 8;

            DrawRectangleRounded(
                {(float)slx, (float)sly0, (float)slw, (float)slh}, 0.08f, 8, {20, 22, 30, 215});
            DrawRectangleRoundedLines({(float)slx, (float)sly0, (float)slw, (float)slh},
                                      0.08f,
                                      8,
                                      1.0f,
                                      {65, 70, 95, 200});

            DrawText("SCENE LEGEND", slx + slp, sly + 2, 11, {150, 155, 180, 255});
            sly += slrh;

            // Helper: draw one labeled row with a colored rectangle swatch
            auto sl_rect = [&](Color c, const char* label) {
                DrawRectangle(slx + slp, sly + 4, 10, 10, c);
                DrawText(label, slx + slp + 18, sly + 2, 11, LIGHTGRAY);
                sly += slrh;
            };
            // Helper: draw one labeled row with a colored circle swatch
            auto sl_circ = [&](Color c, const char* label) {
                DrawCircle(slx + slp + 5, sly + slrh / 2, 5, c);
                DrawText(label, slx + slp + 18, sly + 2, 11, LIGHTGRAY);
                sly += slrh;
            };
            // Helper: draw one labeled row with a short horizontal line swatch
            auto sl_line = [&](Color c, const char* label) {
                DrawLine(slx + slp, sly + slrh / 2, slx + slp + 10, sly + slrh / 2, c);
                DrawText(label, slx + slp + 18, sly + 2, 11, LIGHTGRAY);
                sly += slrh;
            };

            sl_rect({55, 115, 55, 255}, "Launcher");
            sl_circ({220, 50, 50, 255}, "Target");
            sl_line({255, 165, 0, 255}, "Trajectory arc");
            sl_circ(YELLOW, "Apex / Impact");
            sl_circ({0, 220, 220, 255}, "Intercept point");
            sl_line({0, 220, 220, 255}, "Lead vector");
            sl_line({255, 100, 100, 255}, "Target path");
            sl_circ({200, 200, 50, 255}, "Path endpoints");
            // Axes row: three small colored squares
            DrawRectangle(slx + slp, sly + 4, 6, 6, RED);
            DrawRectangle(slx + slp + 8, sly + 4, 6, 6, GREEN);
            DrawRectangle(slx + slp + 16, sly + 4, 6, 6, BLUE);
            DrawText("E / N / Up  axes", slx + slp + 26, sly + 2, 11, LIGHTGRAY);
        }

        // ---- Controls / Help legend overlay ---------------------------------
        if (show_help_legend) {
            const Color lhdr = {150, 155, 180, 255};
            const Color lhc  = {175, 178, 200, 255};
            const Color ldim = {140, 140, 160, 200};
            const int   hlx  = PANEL_W + 16;
            const int   hlw  = 365;
            const int   hlh  = 296;
            const int   hly0 = (H - hlh) / 2;
            const int   hlp  = 12;
            int         hly  = hly0 + 12;

            DrawRectangleRounded(
                {(float)hlx, (float)hly0, (float)hlw, (float)hlh}, 0.06f, 8, {18, 20, 28, 228});
            DrawRectangleRoundedLines({(float)hlx, (float)hly0, (float)hlw, (float)hlh},
                                      0.06f,
                                      8,
                                      1.5f,
                                      {65, 70, 95, 220});

            DrawText("CONTROLS & UI GUIDE", hlx + hlp, hly, 14, WHITE);
            hly += 22;

            DrawText("KEYBOARD CONTROLS", hlx + hlp, hly, 11, lhdr);
            hly += 15;
            DrawText("W / S / A / D      Move target  N / S / E / W", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("Q / E              Move target altitude", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("  (disabled while moving target is active)", hlx + hlp, hly, 10, ldim);
            hly += 14;
            DrawText("Arrow keys         Move launcher  N / S / E / W", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("Page Up / Down     Launcher altitude", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("Right-mouse drag   Orbit camera", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("Middle-mouse drag  Pan camera  (Free mode only)", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("Scroll wheel       Zoom", hlx + hlp, hly, 11, lhc);
            hly += 20;

            DrawText("UI PANEL", hlx + hlp, hly, 11, lhdr);
            hly += 15;
            DrawText(
                "MUNITION       Select projectile type; set muzzle speed", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText(
                "LAUNCHER       Set launcher position  (E / N / Alt m)", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText("TARGET         Set target position", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText(
                "               (locked when moving target is active)", hlx + hlp, hly, 10, ldim);
            hly += 14;
            DrawText("MOVING TARGET  Enable mode; set speed, heading, distance",
                     hlx + hlp,
                     hly,
                     11,
                     lhc);
            hly += 14;
            DrawText(
                "FIRE SOLUTION  Angles, range, flight time, ready status", hlx + hlp, hly, 11, lhc);
            hly += 14;
            DrawText(
                "VIEW FOCUS     Camera follows  Free / Launcher / Target", hlx + hlp, hly, 11, lhc);
        }

        // ---- GUI panel (left side) ----------------------------------------
        DrawRectangle(0, 0, PANEL_W, H, {40, 43, 54, 255});
        DrawRectangle(PANEL_W - 2, 0, 2, H, {65, 70, 95, 255}); // panel border

        const int   mx       = 12;                   // horizontal margin
        const int   cw       = PANEL_W - mx * 2;     // usable control width
        const int   rh       = 26;                   // row height
        const Color sec_col  = {150, 155, 180, 255}; // section label colour
        const Color val_col  = WHITE;
        const Color ctrl_col = LIGHTGRAY;
        int         y        = 12;

        DrawText("Ballistics Renderer", mx, y, 18, WHITE);
        y += 34;

        // ---- Munition -------------------------------------------------------
        DrawText("MUNITION", mx, y, 12, sec_col);
        y += 16;
        const int mun_dd_y = y; // save Y for deferred dropdown draw
        y += rh + 6;

        // Muzzle speed slider
        DrawText("Muzzle speed (m/s)", mx, y, 12, sec_col);
        y += 16;
        GuiSliderBar({(float)mx, (float)y, (float)(cw - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &muzzle_speed,
                     100.f,
                     1500.f);
        DrawText(TextFormat("%.0f", muzzle_speed), mx + cw - 62, y + 6, 14, val_col);
        y += rh + 14;

        // ---- Launcher -------------------------------------------------------
        DrawText("LAUNCHER  (x=East, y=North, z=Alt  metres)", mx, y, 12, sec_col);
        y += 17;

        DrawText("X(E)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &lx,
                     -5000.f,
                     5000.f);
        DrawText(TextFormat("%+.0f", lx), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Y(N)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &ly,
                     -5000.f,
                     5000.f);
        DrawText(TextFormat("%+.0f", ly), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Alt", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &lz,
                     0.f,
                     2000.f);
        DrawText(TextFormat("%.1f", lz), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 14;

        // ---- Target ---------------------------------------------------------
        DrawText("TARGET  (x=East, y=North, z=Alt  metres)", mx, y, 12, sec_col);
        y += 17;

        if (target_moving)
            GuiLock();

        DrawText("X(E)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &tx,
                     -5000.f,
                     5000.f);
        DrawText(TextFormat("%+.0f", tx), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Y(N)", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &ty,
                     -5000.f,
                     5000.f);
        DrawText(TextFormat("%+.0f", ty), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 3;

        DrawText("Alt", mx, y + 6, 13, ctrl_col);
        GuiSliderBar({(float)(mx + 38), (float)y, (float)(cw - 38 - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &tz,
                     0.f,
                     2000.f);
        DrawText(TextFormat("%.1f", tz), mx + cw - 62, y + 6, 13, val_col);
        y += rh + 10;

        if (target_moving)
            GuiUnlock();

        // ---- Moving Target --------------------------------------------------
        DrawText("MOVING TARGET", mx, y, 12, sec_col);
        y += 17;

        GuiCheckBox({(float)mx, (float)y, 20.f, 20.f}, "Enable", &target_moving);
        y += rh + 3;

        DrawText("Speed (m/s)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({(float)mx, (float)(y + 18), (float)(cw - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &target_speed,
                     0.5f,
                     200.f);
        DrawText(TextFormat("%.1f", target_speed), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 22;

        DrawText("Heading (deg)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({(float)mx, (float)(y + 18), (float)(cw - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &target_yaw,
                     0.f,
                     360.f);
        DrawText(TextFormat("%.0f", target_yaw), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 22;

        DrawText("Distance (m)", mx, y + 6, 12, ctrl_col);
        GuiSliderBar({(float)mx, (float)(y + 18), (float)(cw - 68), (float)rh},
                     nullptr,
                     nullptr,
                     &target_travel_dist,
                     10.f,
                     5000.f);
        DrawText(TextFormat("%.0f", target_travel_dist), mx + cw - 62, y + 24, 14, val_col);
        y += rh + 26;

        // ---- Fire Solution --------------------------------------------------
        DrawText("FIRE SOLUTION", mx, y, 12, sec_col);
        y += 18;

        if (current.valid) {
            const Color fc = {75, 215, 100, 255};

            // Commanded (fire-solution) angles
            DrawText(TextFormat("Cmd Az    : %7.2f deg", current.azimuth_deg), mx, y, 14, fc);
            y += 20;
            DrawText(TextFormat("Cmd El    : %7.2f deg", current.elevation_deg), mx, y, 14, fc);
            y += 20;

            // Physical (slewed) angles — show in white; turn green when on-target
            const float az_err =
                std::fabs(std::fmod(phys_az - (float)current.azimuth_deg + 540.f, 360.f) - 180.f);
            const float el_err = std::fabs(phys_el - (float)current.elevation_deg);
            const bool  on_az  = az_err < 0.5f;
            const bool  on_el  = el_err < 0.5f;
            const bool  ready  = on_az && on_el;
            DrawText(
                TextFormat("Phys Az   : %7.2f deg", phys_az), mx, y, 14, on_az ? fc : LIGHTGRAY);
            y += 20;
            DrawText(
                TextFormat("Phys El   : %7.2f deg", phys_el), mx, y, 14, on_el ? fc : LIGHTGRAY);
            y += 20;

            // Ready-to-fire indicator
            if (ready) {
                DrawText("  ** READY TO FIRE **", mx, y, 14, {0, 255, 80, 255});
                y += 20;
            } else {
                DrawText(
                    TextFormat("  Slewing... Az%.1f El%.1f", az_err, el_err), mx, y, 13, YELLOW);
                y += 20;
            }

            DrawText(TextFormat("Range     : %7.0f m", current.range_m), mx, y, 14, fc);
            y += 20;
            DrawText(TextFormat("Flight T  : %7.3f s", current.flight_time_s), mx, y, 14, fc);
            y += 20;

            if (current.has_intercept) {
                const Color lc = {0, 220, 220, 255};
                DrawText(TextFormat("Lead dist : %7.1f m", current.lead_distance_m), mx, y, 14, lc);
                y += 20;
                DrawText(TextFormat("Slew time : %7.2f s", current.slew_time_s), mx, y, 14, lc);
                y += 20;
            }
        } else if (solver.computing()) {
            DrawText("  Computing...", mx, y, 14, YELLOW);
            y += 20;
        } else {
            const Color err_col = {220, 80, 80, 255};
            DrawText("  No solution", mx, y, 14, err_col);
            y += 18;
            DrawText(
                TextFormat("  Range: %.0f m  Max: %.0f m", current.range_m, current.max_range_m),
                mx,
                y,
                12,
                err_col);
            y += 16;
            DrawText(TextFormat("  Alt diff: %+.1f m (launcher - target)", current.alt_diff_m),
                     mx,
                     y,
                     12,
                     err_col);
            y += 16;
        }

        // ---- View Focus -----------------------------------------------------
        y += 4;
        DrawText("VIEW FOCUS", mx, y, 12, sec_col);
        y += 16;
        const int vf_dd_y = y; // save Y for deferred dropdown draw
        y += rh + 6;


        // ---- Deferred dropdown draws (on top of all other controls) ---------
        if (GuiDropdownBox({(float)mx, (float)vf_dd_y, (float)cw, (float)rh},
                           "Free;Launcher;Target",
                           &cam_focus_mode,
                           vf_dd_edit))
            vf_dd_edit = !vf_dd_edit;

        if (GuiDropdownBox({(float)mx, (float)mun_dd_y, (float)cw, (float)rh},
                           dd_str.c_str(),
                           &mun_idx,
                           dd_edit))
            dd_edit = !dd_edit;

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
