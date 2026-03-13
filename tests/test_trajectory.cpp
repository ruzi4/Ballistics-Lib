/// @file test_trajectory.cpp
/// @brief Unit tests for Ballistics-Lib.
///
/// Uses a minimal hand-rolled test framework (no external dependency).
/// Build and run: ctest --output-on-failure

#include <ballistics/ballistics.hpp>
#include <ballistics/math/math_constants.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ballistics;

// ---------------------------------------------------------------------------
// Tiny test framework
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #expr); \
        ++g_fail; \
    } else { \
        ++g_pass; \
    } \
} while (false)

#define CHECK_NEAR(a, b, tol) \
    CHECK(std::abs((a) - (b)) <= (tol))

#define SECTION(name) std::printf("\n[%s]\n", (name))

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a zero-drag munition (Cd = 0) for vacuum trajectory tests.
static MunitionSpec vacuum_munition() {
    MunitionSpec m;
    m.name              = "vacuum_test";
    m.mass_kg           = 0.01;
    m.density_kg_m3     = 11000.0;
    m.reference_area_m2 = 1e-4;
    m.drag_coefficient  = 0.0;  // No drag
    m.diameter_m        = 0.01;
    return m;
}

/// Build a standard munition for drag tests.
static MunitionSpec drag_munition() {
    MunitionSpec m;
    m.name              = "drag_test";
    m.mass_kg           = 0.01;
    m.density_kg_m3     = 11000.0;
    m.reference_area_m2 = 1e-4;
    m.drag_coefficient  = 0.4;
    m.diameter_m        = 0.01;
    return m;
}

/// Standard sea-level atmosphere.
static AtmosphericConditions still_atm() {
    return AtmosphericConditions{};  // default: 1.225 kg/m³
}

// ---------------------------------------------------------------------------
// Vec3 tests
// ---------------------------------------------------------------------------
static void test_vec3() {
    SECTION("Vec3 arithmetic");

    Vec3 a{1, 2, 3};
    Vec3 b{4, 5, 6};

    auto s = a + b;
    CHECK_NEAR(s.x, 5.0, 1e-15);
    CHECK_NEAR(s.y, 7.0, 1e-15);
    CHECK_NEAR(s.z, 9.0, 1e-15);

    auto d = b - a;
    CHECK_NEAR(d.x, 3.0, 1e-15);

    auto scaled = a * 2.0;
    CHECK_NEAR(scaled.x, 2.0, 1e-15);
    CHECK_NEAR(scaled.z, 6.0, 1e-15);

    CHECK_NEAR(a.dot(b), 32.0, 1e-15);
    { const double n = Vec3{3, 0, 0}.norm(); CHECK_NEAR(n, 3.0, 1e-15); }
    { const double n = Vec3{3, 4, 0}.norm(); CHECK_NEAR(n, 5.0, 1e-15); }

    Vec3 u = Vec3{3, 0, 0}.normalized();
    CHECK_NEAR(u.x, 1.0, 1e-15);
    CHECK_NEAR(u.norm(), 1.0, 1e-15);

    // Zero vector normalization should not crash
    Vec3 zero = Vec3{}.normalized();
    CHECK_NEAR(zero.norm(), 0.0, 1e-15);
}

// ---------------------------------------------------------------------------
// Atmosphere tests
// ---------------------------------------------------------------------------
static void test_atmosphere() {
    SECTION("Atmosphere constant model");

    // Default construction yields ISA sea-level density
    AtmosphericConditions atm;
    CHECK_NEAR(atm.air_density_kg_m3, 1.225, 0.001);

    // Custom density is preserved
    AtmosphericConditions thin;
    thin.air_density_kg_m3 = 0.9;
    CHECK_NEAR(thin.air_density_kg_m3, 0.9, 1e-15);
}

// ---------------------------------------------------------------------------
// Munition library tests
// ---------------------------------------------------------------------------
static void test_munition_library() {
    SECTION("MunitionLibrary JSON loading");

    const std::string json = R"({
        "munitions": [
            {
                "name": "test_round",
                "mass_kg": 0.01,
                "density_kg_m3": 11000,
                "reference_area_m2": 1e-4,
                "drag_coefficient": 0.30,
                "muzzle_velocity_ms": 900.0,
                "diameter_m": 0.01
            }
        ]
    })";

    MunitionLibrary lib;
    lib.load_from_string(json);

    CHECK(lib.size() == 1);
    CHECK(lib.contains("test_round"));
    CHECK(!lib.contains("nonexistent"));

    const auto& spec = lib.get("test_round");
    CHECK(spec.name == "test_round");
    CHECK_NEAR(spec.mass_kg,            0.01,    1e-15);
    CHECK_NEAR(spec.density_kg_m3,      11000.0, 1e-10);
    CHECK_NEAR(spec.reference_area_m2,  1e-4,    1e-15);
    CHECK_NEAR(spec.drag_coefficient,   0.30,    1e-15);
    CHECK_NEAR(spec.muzzle_velocity_ms, 900.0,   1e-10);
    CHECK_NEAR(spec.diameter_m,         0.01,    1e-15);

    // Derived quantities
    CHECK_NEAR(spec.ballistic_coefficient(),
               spec.mass_kg / (spec.drag_coefficient * spec.reference_area_m2),
               1e-10);
    CHECK_NEAR(spec.volume_m3(),
               spec.mass_kg / spec.density_kg_m3,
               1e-20);

    // Missing field should throw
    const std::string bad_json = R"({"munitions":[{"name":"x","mass_kg":1.0}]})";
    bool threw = false;
    try { MunitionLibrary b; b.load_from_string(bad_json); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);

    // out_of_range on missing name
    threw = false;
    try { (void)lib.get("ghost"); }
    catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);

    // Bad JSON
    threw = false;
    try { MunitionLibrary b; b.load_from_string("not json at all!"); }
    catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// Vacuum trajectory (zero drag)
// ---------------------------------------------------------------------------
static void test_vacuum_trajectory() {
    SECTION("Vacuum trajectory (Cd=0) — parabolic motion");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(vacuum_munition(), atm);

    // Horizontal shot
    const double v0 = 100.0;  // m/s horizontal
    const double z0 = 100.0;  // m above ground

    ProjectileState state;
    state.position = Vec3{0.0, 0.0, z0};
    state.velocity = Vec3{v0, 0.0, 0.0};
    state.time     = 0.0;

    // Analytical: time to ground = sqrt(2*z0/g)
    const double g      = 9.80665;
    const double t_land = std::sqrt(2.0 * z0 / g);
    const double x_land = v0 * t_land;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 1000.0;
    cfg.max_time = 10.0;
    cfg.use_rk4  = true;

    auto states = sim.simulate(state, cfg);
    CHECK(!states.empty());

    const auto& impact = states.back();
    // Impact time within 1 ms of analytical
    CHECK_NEAR(impact.time, t_land, 0.01);
    // Impact range within 1 m of analytical
    CHECK_NEAR(impact.position.x, x_land, 1.0);
    // z at or near ground_z = 0
    CHECK(impact.position.z <= 0.01);

    // Horizontal velocity unchanged (no drag)
    CHECK_NEAR(impact.velocity.x, v0, 0.1);
}

// ---------------------------------------------------------------------------
// Drag reduces range vs vacuum
// ---------------------------------------------------------------------------
static void test_drag_reduces_range() {
    SECTION("Drag reduces horizontal range");

    AtmosphericConditions atm = still_atm();

    const double v0        = 600.0;  // m/s muzzle velocity
    const double angle_deg = 30.0;
    const double angle_rad = angle_deg * (kDegToRad);

    ProjectileState initial;
    initial.position = Vec3{0.0, 0.0, 0.0};
    initial.velocity = Vec3{v0 * std::cos(angle_rad), 0.0,
                            v0 * std::sin(angle_rad)};
    initial.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 1000.0;
    cfg.max_time = 120.0;
    cfg.use_rk4  = true;

    // Vacuum simulation
    TrajectorySimulator vac_sim(vacuum_munition(), atm);
    auto vac_states = vac_sim.simulate(initial, cfg);

    // Drag simulation
    TrajectorySimulator drag_sim(drag_munition(), atm);
    auto drag_states = drag_sim.simulate(initial, cfg);

    CHECK(!vac_states.empty());
    CHECK(!drag_states.empty());

    const double x_vac  = vac_states.back().position.x;
    const double x_drag = drag_states.back().position.x;

    std::printf("  Vacuum range: %.1f m,  Drag range: %.1f m\n", x_vac, x_drag);
    CHECK(x_drag < x_vac);
    // Drag should remove at least 30% of range at these parameters
    CHECK(x_drag < 0.7 * x_vac);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Real-time step performance
// ---------------------------------------------------------------------------
static void test_realtime_performance() {
    SECTION("Real-time step performance");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    ProjectileState state;
    state.position = Vec3{0.0, 0.0, 1.5};
    state.velocity = Vec3{800.0, 0.0, 400.0};
    state.time     = 0.0;

    // Simulate 1 second at 240 Hz (= 240 steps) and measure total time
    const double dt         = 1.0 / 240.0;
    const int    num_steps  = 240;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_steps; ++i)
        state = sim.step(state, dt);
    auto t1 = std::chrono::high_resolution_clock::now();

    const double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double per_step_us = total_us / num_steps;
    // Budget per 60 Hz frame: 16 667 µs
    // At 4 sub-steps per frame the per-step budget is 4 167 µs — very generous.
    // We require < 100 µs per step to leave ample headroom for many projectiles.
    std::printf("  %d RK4 steps in %.1f µs (%.2f µs/step)\n",
                num_steps, total_us, per_step_us);
    CHECK(per_step_us < 100.0);  // should be <<1 µs on any modern hardware
}

// ---------------------------------------------------------------------------
// Streaming callback
// ---------------------------------------------------------------------------
static void test_streaming_callback() {
    SECTION("Streaming simulate() callback");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    ProjectileState initial;
    initial.position = Vec3{0.0, 0.0, 1.0};
    initial.velocity = Vec3{300.0, 0.0, 200.0};
    initial.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 30.0;

    // Count states via callback and compare to batch
    int  cb_count = 0;
    bool monotone = true;
    double prev_time = -1.0;
    sim.simulate(initial, cfg, [&](const ProjectileState& s) {
        ++cb_count;
        if (s.time < prev_time) monotone = false;
        prev_time = s.time;
        return true;
    });

    auto states = sim.simulate(initial, cfg);
    CHECK(cb_count == static_cast<int>(states.size()));
    CHECK(monotone);
    CHECK(cb_count > 0);

    // Early stop
    int stop_at = 10;
    int counted = 0;
    sim.simulate(initial, cfg, [&](const ProjectileState&) {
        return ++counted < stop_at;
    });
    CHECK(counted == stop_at);
}

// ---------------------------------------------------------------------------
// Fire control tests
// ---------------------------------------------------------------------------
static void test_fire_control_basic() {
    SECTION("solve_elevation: low-angle solution lands at requested range");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    const double muzzle_speed = 600.0;   // m/s
    const double target_range = 500.0;   // m
    const double az           = 0.0;     // due North

    FireSolution sol = solve_elevation(sim,
                                       LauncherOrientation{az},
                                       target_range,
                                       muzzle_speed);
    CHECK(sol.valid);
    CHECK(sol.elevation_deg > 0.0);
    CHECK(sol.elevation_deg < 45.0);   // low-angle solution
    CHECK(sol.flight_time_ms > 0.0);

    // Verify the solution: re-simulate with the returned elevation angle
    const double el = sol.elevation_deg * (kDegToRad);
    const double az_rad = az * (kDegToRad);

    ProjectileState initial;
    initial.position = {};
    initial.velocity = muzzle_speed * Vec3{std::cos(el) * std::sin(az_rad),
                                           std::cos(el) * std::cos(az_rad),
                                           std::sin(el)};
    initial.time = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 300.0;

    auto states = sim.simulate(initial, cfg);
    CHECK(!states.empty());

    const auto& impact = states.back();
    const double actual_range = std::sqrt(impact.position.x * impact.position.x
                                        + impact.position.y * impact.position.y);
    std::printf("  Requested: %.1f m  |  Actual: %.2f m  |  Elev: %.3f°  |  ToF: %.1f ms\n",
                target_range, actual_range, sol.elevation_deg, sol.flight_time_ms);

    // Solution should hit within 0.5 m of the requested range
    CHECK_NEAR(actual_range, target_range, 0.5);

    // Flight time should agree with simulation to within 5 ms
    CHECK_NEAR(sol.flight_time_ms, impact.time * 1000.0, 5.0);
}

static void test_fire_control_high_angle() {
    SECTION("solve_elevation: high-angle solution lands at same range, longer ToF");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    const double muzzle_speed = 600.0;
    const double target_range = 300.0;
    const double az           = 90.0;  // due East

    FireSolution lo = solve_elevation(sim, LauncherOrientation{az},
                                       target_range, muzzle_speed,
                                       0.0, /*high_angle=*/false);
    FireSolution hi = solve_elevation(sim, LauncherOrientation{az},
                                       target_range, muzzle_speed,
                                       0.0, /*high_angle=*/true);

    CHECK(lo.valid);
    CHECK(hi.valid);

    std::printf("  Low-angle : elev=%.2f°  ToF=%.0f ms\n",
                lo.elevation_deg, lo.flight_time_ms);
    std::printf("  High-angle: elev=%.2f°  ToF=%.0f ms\n",
                hi.elevation_deg, hi.flight_time_ms);

    // High-angle solution must have a steeper elevation and longer flight time
    CHECK(hi.elevation_deg > lo.elevation_deg);
    CHECK(hi.flight_time_ms > lo.flight_time_ms);
}

static void test_fire_control_out_of_range() {
    SECTION("solve_elevation: returns valid=false when target is out of range");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    // 1 m/s muzzle velocity — cannot reach 10 000 m
    FireSolution sol = solve_elevation(sim, LauncherOrientation{0.0},
                                       10000.0, /*muzzle_speed=*/1.0);
    CHECK(!sol.valid);
}

static void test_fire_control_elevated_launcher() {
    SECTION("solve_elevation: launcher above target plane");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    const double muzzle_speed    = 400.0;
    const double target_range    = 400.0;
    const double launch_height   = 100.0;  // launcher 100 m above target

    FireSolution sol = solve_elevation(sim, LauncherOrientation{0.0},
                                       target_range, muzzle_speed,
                                       launch_height);
    CHECK(sol.valid);

    std::printf("  Elevated launcher (h=%.0fm): elev=%.3f°  ToF=%.0f ms\n",
                launch_height, sol.elevation_deg, sol.flight_time_ms);

    // With a 100 m height advantage, elevation can be negative (shooting down)
    // For a 400 m range this should be sub-horizontal
    CHECK(sol.elevation_deg < 45.0);
}

static void test_fire_control_azimuth_independence() {
    SECTION("solve_elevation: elevation angle is azimuth-independent");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    const double muzzle_speed = 500.0;
    const double range        = 350.0;

    FireSolution north = solve_elevation(sim, LauncherOrientation{  0.0}, range, muzzle_speed);
    FireSolution east  = solve_elevation(sim, LauncherOrientation{ 90.0}, range, muzzle_speed);
    FireSolution south = solve_elevation(sim, LauncherOrientation{180.0}, range, muzzle_speed);

    CHECK(north.valid && east.valid && south.valid);

    std::printf("  N: %.4f°  E: %.4f°  S: %.4f°\n",
                north.elevation_deg, east.elevation_deg, south.elevation_deg);

    // Elevation should be identical regardless of azimuth
    CHECK_NEAR(north.elevation_deg, east.elevation_deg,  0.01);
    CHECK_NEAR(north.elevation_deg, south.elevation_deg, 0.01);
}

// ---------------------------------------------------------------------------
// FireControlTable tests
// ---------------------------------------------------------------------------
static void test_table_builds_and_is_ready() {
    SECTION("FireControlTable: builds without error and marks ready");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    FireControlTable table;
    CHECK(!table.ready());
    CHECK(table.max_range_m() == 0.0);

    table.build(sim, 600.0);

    CHECK(table.ready());
    CHECK(table.max_range_m() > 0.0);
    CHECK(!table.entries().empty());

    std::printf("  max_range=%.1f m  entries=%zu\n",
                table.max_range_m(), table.entries().size());
}

static void test_table_lookup_accuracy() {
    SECTION("FireControlTable: lookup agrees with solve_elevation within 1 m");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    FireControlTable table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);

    CHECK(table.ready());

    // Test several ranges across the achievable envelope
    const double max_r = table.max_range_m();
    for (double frac : {0.1, 0.3, 0.5, 0.7, 0.9}) {
        const double range = frac * max_r;
        FireSolution ref = solve_elevation(sim, LauncherOrientation{0.0},
                                           range, 600.0);
        FireSolution tbl = table.lookup(range);

        CHECK(tbl.valid);
        CHECK(ref.valid);

        // Elevation should agree within 0.2°
        CHECK_NEAR(tbl.elevation_deg, ref.elevation_deg, 0.2);
        // Flight time should agree within 50 ms
        CHECK_NEAR(tbl.flight_time_ms, ref.flight_time_ms, 50.0);
    }
}

static void test_table_out_of_range() {
    SECTION("FireControlTable: lookup returns valid=false beyond max range");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    FireControlTable table;
    table.build(sim, 600.0);

    FireSolution beyond = table.lookup(table.max_range_m() * 2.0);
    CHECK(!beyond.valid);

    FireSolution negative = table.lookup(-10.0);
    CHECK(!negative.valid);
}

static void test_table_lookup_realtime_performance() {
    SECTION("FireControlTable: lookup is fast enough for 60 Hz game loop");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    FireControlTable table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);

    CHECK(table.ready());

    const double max_r    = table.max_range_m();
    const int    N        = 10000;  // simulate 10 000 consecutive frame lookups

    auto t0 = std::chrono::high_resolution_clock::now();
    volatile double sink = 0.0;  // prevent the compiler optimising the loop away
    for (int i = 0; i < N; ++i) {
        const double range = max_r * (i % 1000) / 1000.0;
        FireSolution s = table.lookup(range);
        sink += s.elevation_deg;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)sink;

    const double total_us  = std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double per_us    = total_us / N;
    const double budget_us = 1e6 / 60.0;   // 16 667 µs per frame

    std::printf("  %d lookups in %.1f µs  (%.3f µs/lookup,  budget=%.0f µs/frame)\n",
                N, total_us, per_us, budget_us);

    // Each lookup must be well under 1 µs — negligible vs 60 Hz budget
    CHECK(per_us < 1.0);
}

static void test_table_build_timing() {
    SECTION("FireControlTable: build time is reported (informational)");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    auto t0 = std::chrono::high_resolution_clock::now();
    FireControlTable table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);
    auto t1 = std::chrono::high_resolution_clock::now();

    const double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("  500-sample table built in %.1f ms  "
                "(run async or during loading)\n", build_ms);

    CHECK(table.ready());
    // Build must complete in < 5 s on any reasonable machine
    CHECK(build_ms < 5000.0);
}

// ---------------------------------------------------------------------------
// Additional tests added to address review findings
// ---------------------------------------------------------------------------

static void test_table_high_angle() {
    SECTION("FireControlTable: high-angle build and lookup");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    FireControlTable table;
    CHECK(!table.ready());

    table.build(sim, 600.0, 0.0, 0.0, /*high_angle=*/true, 500);

    CHECK(table.ready());
    CHECK(table.max_range_m() > 0.0);
    CHECK(!table.entries().empty());

    // Verify a mid-range lookup is valid and has a high elevation angle
    const double test_range = table.max_range_m() * 0.5;
    FireSolution sol = table.lookup(test_range);
    CHECK(sol.valid);
    // High-angle solution must be above 45°
    CHECK(sol.elevation_deg > 45.0);
    CHECK(sol.flight_time_ms > 0.0);

    std::printf("  High-angle table: max_range=%.1f m  entries=%zu\n",
                table.max_range_m(), table.entries().size());
    std::printf("  Lookup at 50%% range: elev=%.2f°  ToF=%.0f ms\n",
                sol.elevation_deg, sol.flight_time_ms);
}

static void test_table_lookup_below_min_range() {
    SECTION("FireControlTable: lookup below minimum entry returns valid=false");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    // High-angle table: near-vertical shots produce short ranges but not
    // exactly 0, so the first entry has a meaningful positive minimum range.
    FireControlTable table;
    table.build(sim, 600.0, 0.0, 0.0, /*high_angle=*/true, 100);
    CHECK(table.ready());

    const double min_range = table.entries().front().range_m;
    std::printf("  High-angle table minimum range: %.2f m\n", min_range);

    // Any range below the minimum entry must return invalid
    if (min_range > 1.0) {
        FireSolution below = table.lookup(min_range * 0.5);
        CHECK(!below.valid);
        std::printf("  lookup(%.2f) = valid=%s (expected false)\n",
                    min_range * 0.5, below.valid ? "true" : "false");
    }

    // Negative range must always be invalid
    FireSolution neg = table.lookup(-1.0);
    CHECK(!neg.valid);

    // Exact minimum entry range must be valid
    FireSolution exact = table.lookup(min_range);
    CHECK(exact.valid);
}


static void test_munition_library_load_file() {
    SECTION("MunitionLibrary: load() from file path");

    MunitionLibrary lib;
    // Try a few candidate paths since the working directory varies by runner
    const char* candidates[] = {
        "data/munitions.json",
        "../data/munitions.json",
        "../../data/munitions.json"
    };
    bool loaded = false;
    for (const char* path : candidates) {
        try {
            lib.load(path);
            loaded = true;
            std::printf("  Loaded from: %s\n", path);
            break;
        } catch (const std::runtime_error&) {}
    }
    if (!loaded)
        std::printf("  Skipped: data/munitions.json not found in any candidate path\n");

    if (loaded) {
        CHECK(lib.size() > 0);
        std::printf("  Loaded %zu munition(s) from file\n", lib.size());
        for (const auto& name : lib.names())
            std::printf("    - %s\n", name.c_str());
    }

    // Non-existent file must throw runtime_error
    bool threw = false;
    try { MunitionLibrary b; b.load("/tmp/does_not_exist_xyz.json"); }
    catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
}

static void test_table_with_target_altitude() {
    SECTION("FireControlTable: build with non-zero target_altitude_m");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    // Build two tables with the same launch height but different target
    // altitudes. Firing down at a depressed target should achieve longer
    // horizontal range than firing at the same elevation.
    const double muzzle   = 600.0;
    const double lh       = 100.0;   // launcher 100 m above sea level

    FireControlTable table_flat;   // target at z = 0
    FireControlTable table_elev;   // target at z = 50 m (half-way up)

    table_flat.build(sim, muzzle, 0.0, lh, false, 200, /*target_altitude_m=*/0.0);
    table_elev.build(sim, muzzle, 0.0, lh, false, 200, /*target_altitude_m=*/50.0);

    CHECK(table_flat.ready());
    CHECK(table_elev.ready());

    // An elevated target means the projectile hits a ground plane that is
    // closer in the vertical dimension, so max horizontal range should differ.
    std::printf("  max_range flat=%.1f m  elevated_target=%.1f m\n",
                table_flat.max_range_m(), table_elev.max_range_m());

    // Both tables should produce valid solutions at the same test range
    const double test_range = std::min(table_flat.max_range_m(),
                                       table_elev.max_range_m()) * 0.5;
    CHECK(table_flat.lookup(test_range).valid);
    CHECK(table_elev.lookup(test_range).valid);
}


static void test_build_num_samples_guard() {
    SECTION("FireControlTable: num_samples < 2 is clamped to 2");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    // Passing 0 or 1 must not crash and must produce a usable table
    for (int bad_n : {0, 1}) {
        FireControlTable table;
        table.build(sim, 500.0, 0.0, 0.0, false, bad_n);
        CHECK(table.ready());
        // A 2-entry table should at least cover some range
        CHECK(table.max_range_m() > 0.0);
        std::printf("  num_samples=%d → entries=%zu  max_range=%.1f m\n",
                    bad_n, table.entries().size(), table.max_range_m());
    }
}

static void test_invalid_muzzle_speed() {
    SECTION("solve_elevation and build: throw on muzzle_speed_ms <= 0");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator sim(drag_munition(), atm);

    // solve_elevation
    bool threw = false;
    try { solve_elevation(sim, LauncherOrientation{0.0}, 100.0, /*muzzle=*/0.0); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);

    threw = false;
    try { solve_elevation(sim, LauncherOrientation{0.0}, 100.0, /*muzzle=*/-1.0); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);

    // FireControlTable::build
    threw = false;
    try { FireControlTable t; t.build(sim, /*muzzle=*/0.0); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);

    threw = false;
    try { FireControlTable t; t.build(sim, /*muzzle=*/-500.0); }
    catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// Altitude variation tests
// ---------------------------------------------------------------------------

// Helper: re-simulate with a given elevation and azimuth, return impact
// (horizontal range, altitude).  The initial position is at absolute altitude
// (target_alt + launch_height).  The ground plane is target_alt.
struct ImpactResult { double h_range_m; double z_m; double time_s; };

static ImpactResult resimulate(const TrajectorySimulator& sim,
                                double elevation_deg,
                                double azimuth_deg,
                                double muzzle_speed_ms,
                                double launch_height_m,
                                double target_alt_m)
{
    const double el    = elevation_deg * kDegToRad;
    const double az    = azimuth_deg   * kDegToRad;
    const Vec3   h_dir = {std::sin(az), std::cos(az), 0.0};

    ProjectileState st;
    st.position = Vec3{0.0, 0.0, target_alt_m + launch_height_m};
    st.velocity = muzzle_speed_ms * (std::cos(el) * h_dir + Vec3{0.0, 0.0, std::sin(el)});
    st.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 120.0;
    cfg.ground_z = target_alt_m;
    cfg.use_rk4  = true;

    ProjectileState last = st;
    sim.simulate(st, cfg, [&last](const ProjectileState& s) {
        last = s;
        return true;
    });

    const double hr = std::sqrt(last.position.x * last.position.x
                               + last.position.y * last.position.y);
    return {hr, last.position.z, last.time};
}

// ---------------------------------------------------------------------------
// Launcher significantly above target — various non-similar altitudes
// ---------------------------------------------------------------------------
static void test_altitude_launcher_above_target() {
    SECTION("Altitude: launcher above target, various altitude pairs");

    // Use 5.56x45 and 7.62x51 from the munitions file
    MunitionLibrary lib;
    bool loaded = false;
    for (const char* p : {"data/munitions.json",
                           "../data/munitions.json",
                           "../../data/munitions.json"}) {
        try { lib.load(p); loaded = true; break; }
        catch (const std::runtime_error&) {}
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Scenario table: {launcher_alt_m, target_alt_m, munition_name, muzzle_ms, range_m}
    // All ranges are well within the munition's achievable envelope.
    // Altitude differences are deliberately large and non-similar.
    struct Scenario {
        double launcher_alt;
        double target_alt;
        const char* munition;
        double muzzle;
        double range;
        const char* label;
    };
    const Scenario scenarios[] = {
        // 5.56 — launcher high above target
        { 600.0,   0.0, "5.56x45_m855_62gr", 930.0, 400.0,
          "5.56: launcher 600m, target 0m, range 400m" },
        { 1200.0, 200.0, "5.56x45_m855_62gr", 930.0, 500.0,
          "5.56: launcher 1200m, target 200m, range 500m" },
        { 800.0,  50.0, "5.56x45_m855_62gr", 930.0, 350.0,
          "5.56: launcher 800m, target 50m, range 350m" },
        // 7.62 — different altitude pairs
        { 500.0,   0.0, "7.62x51_m80_147gr", 853.0, 400.0,
          "7.62: launcher 500m, target 0m, range 400m" },
        { 1000.0, 300.0, "7.62x51_m80_147gr", 853.0, 500.0,
          "7.62: launcher 1000m, target 300m, range 500m" },
        // .338 Lapua — large altitude differences
        { 1500.0, 100.0, "338_lapua_250gr", 930.0, 700.0,
          ".338: launcher 1500m, target 100m, range 700m" },
        { 2000.0, 500.0, "338_lapua_250gr", 930.0, 800.0,
          ".338: launcher 2000m, target 500m, range 800m" },
    };

    for (const auto& s : scenarios) {
        const double launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm;
        const MunitionSpec& spec = lib.get(s.munition);
        TrajectorySimulator sim(spec, atm);

        FireSolution sol = solve_elevation(sim,
                                           LauncherOrientation{0.0},
                                           s.range,
                                           s.muzzle,
                                           launch_height,
                                           /*high_angle=*/false,
                                           /*tolerance_m=*/0.5,
                                           s.target_alt);
        std::printf("  [%s]\n    elev=%.3f°  ToF=%.0f ms  valid=%s\n",
                    s.label, sol.elevation_deg, sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid) continue;

        // Re-simulate and verify the solution
        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0,
                                      s.muzzle, launch_height, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m, imp.z_m, s.target_alt);
        // Impact horizontal range must be within 1 m of requested range
        CHECK_NEAR(imp.h_range_m, s.range, 1.0);
        // Impact altitude must be at target plane within 1 m
        CHECK_NEAR(imp.z_m, s.target_alt, 1.0);
    }
}

// ---------------------------------------------------------------------------
// Launcher below target — two sub-tests:
//
// Part A: impossible small-range cases.  The fire control system finds target
//         impact via the DESCENDING crossing of target_z_m.  When the launcher
//         is below the target the minimum achievable range on the descending
//         path can be several km, far larger than a direct-line shot.  Requests
//         for ranges shorter than that minimum must return valid=false.
//
// Part B: achievable long-range cases.  The range is determined dynamically
//         from the FireControlTable envelope and re-simulated to verify accuracy.
// ---------------------------------------------------------------------------
static void test_altitude_launcher_below_target() {
    SECTION("Altitude: launcher below target — impossible small ranges → valid=false");

    MunitionLibrary lib;
    bool loaded = false;
    for (const char* p : {"data/munitions.json",
                           "../data/munitions.json",
                           "../../data/munitions.json"}) {
        try { lib.load(p); loaded = true; break; }
        catch (const std::runtime_error&) {}
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // These ranges are far below the minimum achievable descending-crossing
    // range for each scenario; valid=false is the correct answer.
    struct ImpossibleCase {
        double launcher_alt, target_alt, muzzle, range;
        const char* munition;
        const char* label;
    };
    const ImpossibleCase impossible[] = {
        { 0.0,  200.0, 930.0, 300.0, "5.56x45_m855_62gr",
          "5.56: launcher 0m, target 200m, range 300m (too short)" },
        { 0.0,  400.0, 853.0, 350.0, "7.62x51_m80_147gr",
          "7.62: launcher 0m, target 400m, range 350m (too short)" },
        { 0.0,  300.0, 930.0, 200.0, "338_lapua_250gr",
          ".338: launcher 0m, target 300m, range 200m (too short)" },
    };

    for (const auto& s : impossible) {
        const double lh = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm;
        TrajectorySimulator sim(lib.get(s.munition), atm);

        FireSolution sol = solve_elevation(sim, LauncherOrientation{0.0},
                                           s.range, s.muzzle,
                                           lh, false, 0.5, s.target_alt);
        std::printf("  [%s]\n    valid=%s (expected false)\n",
                    s.label, sol.valid ? "true" : "false");
        CHECK(!sol.valid);
    }

    // -----------------------------------------------------------------------
    SECTION("Altitude: launcher below target — achievable ranges (high-angle) → valid=true");

    // When the launcher is below the target the projectile must arc above
    // the target plane and descend onto it — this is inherently high-angle
    // (plunging fire).  The low-angle regime has no solutions here because
    // every angle < theta_max leaves the projectile below target_z_m on
    // the descending branch.  Callers must set high_angle=true.
    //
    // We determine the achievable range from a coarse high-angle table and
    // test at 50 % of the maximum range.
    struct AchievableCase {
        double launcher_alt, target_alt, muzzle;
        const char* munition;
        const char* label;
    };
    const AchievableCase achievable[] = {
        { 0.0,  100.0, 930.0, "5.56x45_m855_62gr",
          "5.56 (high-angle): launcher 0m, target 100m" },
        { 100.0, 300.0, 853.0, "7.62x51_m80_147gr",
          "7.62 (high-angle): launcher 100m, target 300m" },
        { 0.0,  200.0, 930.0, "338_lapua_250gr",
          ".338 (high-angle): launcher 0m, target 200m" },
        { 200.0, 600.0, 930.0, "338_lapua_250gr",
          ".338 (high-angle): launcher 200m, target 600m" },
    };

    for (const auto& s : achievable) {
        const double lh = s.launcher_alt - s.target_alt;  // negative
        AtmosphericConditions atm;
        TrajectorySimulator sim(lib.get(s.munition), atm);

        // Use a coarse HIGH-ANGLE table to find the achievable range envelope
        FireControlTable probe;
        probe.build(sim, s.muzzle, 0.0, lh,
                    /*high_angle=*/true, 80, s.target_alt);

        if (!probe.ready() || probe.max_range_m() < 200.0) {
            std::printf("  [%s] Skipped: max_range=%.0f m too small\n",
                        s.label, probe.max_range_m());
            continue;
        }

        const double test_range = probe.max_range_m() * 0.5;

        // Must use high_angle=true: plunging trajectory is the only option
        // when the launcher is below the target plane.
        FireSolution sol = solve_elevation(sim, LauncherOrientation{0.0},
                                           test_range, s.muzzle,
                                           lh, /*high_angle=*/true,
                                           0.5, s.target_alt);
        std::printf("  [%s]\n    max_range=%.0f m  test_range=%.0f m\n"
                    "    elev=%.3f°  ToF=%.0f ms  valid=%s\n",
                    s.label, probe.max_range_m(), test_range,
                    sol.elevation_deg, sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid) continue;
        // High-angle plunging shot: elevation is positive (fires upward)
        CHECK(sol.elevation_deg > 0.0);

        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0,
                                      s.muzzle, lh, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m, imp.z_m, s.target_alt);
        CHECK_NEAR(imp.h_range_m, test_range, 2.0);
        CHECK_NEAR(imp.z_m, s.target_alt, 1.0);
    }
}

// ---------------------------------------------------------------------------
// Both launcher and target at non-zero altitudes (various non-similar pairs)
// ---------------------------------------------------------------------------
static void test_altitude_both_nonzero() {
    SECTION("Altitude: both launcher and target at non-zero altitudes");

    MunitionLibrary lib;
    bool loaded = false;
    for (const char* p : {"data/munitions.json",
                           "../data/munitions.json",
                           "../../data/munitions.json"}) {
        try { lib.load(p); loaded = true; break; }
        catch (const std::runtime_error&) {}
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    struct Scenario {
        double launcher_alt;
        double target_alt;
        const char* munition;
        double muzzle;
        double range;  // 0 = determine dynamically from FireControlTable
        const char* label;
    };
    const Scenario scenarios[] = {
        // Launcher above target, both non-zero — ranges are well within envelope
        { 800.0,  200.0, "5.56x45_m855_62gr", 930.0, 400.0,
          "5.56: launcher 800m, target 200m (above), range 400m" },
        { 1500.0, 600.0, "7.62x51_m80_147gr", 853.0, 500.0,
          "7.62: launcher 1500m, target 600m (above), range 500m" },
        // Launcher below target — use range=0 to trigger dynamic determination
        { 300.0,  900.0, "338_lapua_250gr", 930.0, 0.0,
          ".338: launcher 300m, target 900m (below), dynamic range" },
        { 100.0,  500.0, "338_lapua_250gr", 930.0, 0.0,
          ".338: launcher 100m, target 500m (below), dynamic range" },
    };

    for (const auto& s : scenarios) {
        const double launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm;
        const MunitionSpec& spec = lib.get(s.munition);
        TrajectorySimulator sim(spec, atm);

        double test_range = s.range;
        // Launcher-below-target requires high-angle (plunging) fire:
        // the simulation only detects descending crossings of target_z_m.
        const bool below_target = (s.launcher_alt < s.target_alt);

        if (test_range <= 0.0) {
            // Determine the achievable range dynamically
            FireControlTable probe;
            probe.build(sim, s.muzzle, 45.0, launch_height,
                        /*high_angle=*/true, 80, s.target_alt);
            if (!probe.ready() || probe.max_range_m() < 200.0) {
                std::printf("  [%s] Skipped: max_range=%.0f m too small\n",
                            s.label, probe.max_range_m());
                continue;
            }
            test_range = probe.max_range_m() * 0.5;
        }

        FireSolution sol = solve_elevation(sim,
                                           LauncherOrientation{45.0},
                                           test_range,
                                           s.muzzle,
                                           launch_height,
                                           /*high_angle=*/below_target,
                                           /*tolerance_m=*/0.5,
                                           s.target_alt);
        std::printf("  [%s]\n    range=%.0f m  elev=%.3f°  ToF=%.0f ms  valid=%s\n",
                    s.label, test_range, sol.elevation_deg, sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid) continue;

        ImpactResult imp = resimulate(sim, sol.elevation_deg, 45.0,
                                      s.muzzle, launch_height, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m, imp.z_m, s.target_alt);
        CHECK_NEAR(imp.h_range_m, test_range, 2.0);
        CHECK_NEAR(imp.z_m, s.target_alt, 1.0);
    }
}

// ---------------------------------------------------------------------------
// FireControlTable with altitude variation
// ---------------------------------------------------------------------------
static void test_altitude_fire_table() {
    SECTION("Altitude: FireControlTable with non-similar launcher/target altitudes");

    MunitionLibrary lib;
    bool loaded = false;
    for (const char* p : {"data/munitions.json",
                           "../data/munitions.json",
                           "../../data/munitions.json"}) {
        try { lib.load(p); loaded = true; break; }
        catch (const std::runtime_error&) {}
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Launcher well above target — elevation is negative (shooting downward)
    {
        const double launcher_alt  = 700.0;
        const double target_alt    = 0.0;
        const double launch_height = launcher_alt - target_alt;  // +700
        AtmosphericConditions atm;
        TrajectorySimulator sim(lib.get("7.62x51_m80_147gr"), atm);

        FireControlTable table;
        table.build(sim, 853.0, 0.0, launch_height, false, 300, target_alt);
        CHECK(table.ready());
        CHECK(table.max_range_m() > 0.0);

        const double test_range = table.max_range_m() * 0.4;
        FireSolution sol = table.lookup(test_range);
        CHECK(sol.valid);
        // Launcher is 700 m above the target; a downward shot is expected.
        // Elevation may legitimately be negative here — do NOT check > 0.
        std::printf("  Table (launcher 700m, target 0m): "
                    "max=%.1f m, lookup@40%%=%.1f m → elev=%.2f°\n",
                    table.max_range_m(), test_range, sol.elevation_deg);

        // Re-simulate to verify the table's interpolated solution
        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0,
                                      853.0, launch_height, target_alt);
        CHECK_NEAR(imp.h_range_m, test_range, test_range * 0.02);  // 2 % tolerance
        CHECK_NEAR(imp.z_m, target_alt, 1.0);
    }

    // Launcher below target — elevation must be positive (shooting upward)
    {
        const double launcher_alt  = 0.0;
        const double target_alt    = 200.0;
        const double launch_height = launcher_alt - target_alt;   // -200
        AtmosphericConditions atm;
        TrajectorySimulator sim(lib.get("338_lapua_250gr"), atm);

        FireControlTable table;
        table.build(sim, 930.0, 0.0, launch_height, /*high_angle=*/true, 300, target_alt);
        CHECK(table.ready());
        CHECK(table.max_range_m() > 0.0);

        // Use 50 % of the achievable max — guaranteed to be in-range
        const double test_range = table.max_range_m() * 0.5;
        FireSolution sol = table.lookup(test_range);
        CHECK(sol.valid);
        // Must shoot upward to arc over and hit the elevated ground plane
        CHECK(sol.elevation_deg > 0.0);
        std::printf("  Table (launcher 0m, target 200m): "
                    "max=%.1f m, lookup@50%%=%.1f m → elev=%.2f°\n",
                    table.max_range_m(), test_range, sol.elevation_deg);

        // Re-simulate to verify
        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0,
                                      930.0, launch_height, target_alt);
        CHECK_NEAR(imp.h_range_m, test_range, test_range * 0.02);
        CHECK_NEAR(imp.z_m, target_alt, 1.0);
    }
}

// ---------------------------------------------------------------------------
// max_time adequacy: verify that altitude scenarios don't trip the 60 s ceiling
// ---------------------------------------------------------------------------
static void test_altitude_flight_time_within_max() {
    SECTION("Altitude: returned flight times are reasonable (< 60 s)");

    MunitionLibrary lib;
    bool loaded = false;
    for (const char* p : {"data/munitions.json",
                           "../data/munitions.json",
                           "../../data/munitions.json"}) {
        try { lib.load(p); loaded = true; break; }
        catch (const std::runtime_error&) {}
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Only use launcher-above-target scenarios here: launcher-below at small
    // ranges are physically impossible and correctly return valid=false.
    // Their flight times are therefore not meaningful to test.
    struct Scenario {
        double launcher_alt, target_alt, muzzle, range;
        const char* munition;
    };
    const Scenario scenarios[] = {
        {  600.0,   0.0, 930.0, 400.0, "5.56x45_m855_62gr" },
        { 1000.0, 100.0, 853.0, 500.0, "7.62x51_m80_147gr" },
        { 2000.0, 300.0, 930.0, 700.0, "338_lapua_250gr"   },
        {  500.0,  50.0, 853.0, 350.0, "7.62x51_m80_147gr" },
    };

    for (const auto& s : scenarios) {
        const double launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm;
        TrajectorySimulator sim(lib.get(s.munition), atm);

        FireSolution sol = solve_elevation(sim,
                                           LauncherOrientation{0.0},
                                           s.range, s.muzzle,
                                           launch_height,
                                           false, 0.5, s.target_alt);
        CHECK(sol.valid);
        if (sol.valid) {
            // Flight time must be well under the 60 000 ms ceiling
            CHECK(sol.flight_time_ms < 60000.0);
            std::printf("  launcher=%.0fm target=%.0fm range=%.0fm → "
                        "elev=%.2f° ToF=%.0f ms\n",
                        s.launcher_alt, s.target_alt, s.range,
                        sol.elevation_deg, sol.flight_time_ms);
        }
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    std::printf("=== Ballistics-Lib Unit Tests ===\n");

    test_vec3();
    test_atmosphere();
    test_munition_library();
    test_vacuum_trajectory();
    test_drag_reduces_range();
    test_realtime_performance();
    test_streaming_callback();
    test_fire_control_basic();

    test_fire_control_high_angle();
    test_fire_control_out_of_range();
    test_fire_control_elevated_launcher();
    test_fire_control_azimuth_independence();
    test_table_builds_and_is_ready();
    test_table_lookup_accuracy();
    test_table_out_of_range();
    test_table_lookup_realtime_performance();
    test_table_build_timing();
    test_table_high_angle();
    test_table_lookup_below_min_range();
    test_munition_library_load_file();
    test_table_with_target_altitude();
    test_build_num_samples_guard();
    test_invalid_muzzle_speed();

    // Altitude variation tests
    test_altitude_launcher_above_target();
    test_altitude_launcher_below_target();
    test_altitude_both_nonzero();
    test_altitude_fire_table();
    test_altitude_flight_time_within_max();

    std::printf("\n================================\n");
    std::printf("Passed: %d   Failed: %d\n", g_pass, g_fail);

    return (g_fail == 0) ? 0 : 1;
}
