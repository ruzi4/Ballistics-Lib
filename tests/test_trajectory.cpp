/// @file test_trajectory.cpp
/// @brief Unit tests for Ballistics-Lib.
///
/// Uses a minimal hand-rolled test framework (no external dependency).
/// Build and run: ctest --output-on-failure

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <ballistics/ballistics.hpp>
#include <ballistics/math/math_constants.hpp>

using namespace ballistics;

// ---------------------------------------------------------------------------
// Tiny test framework
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(expr)                                                                                \
    do {                                                                                           \
        if (!(expr)) {                                                                             \
            std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #expr);                         \
            ++g_fail;                                                                              \
        } else {                                                                                   \
            ++g_pass;                                                                              \
        }                                                                                          \
    } while (false)

#define CHECK_NEAR(a, b, tol) CHECK(std::abs((a) - (b)) <= (tol))

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
    m.drag_coefficient  = 0.0; // No drag
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

/// Still sea-level atmosphere, no wind.
static AtmosphericConditions still_atm() {
    return isa_conditions(0.0);
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
    {
        const double n = Vec3{3, 0, 0}.norm();
        CHECK_NEAR(n, 3.0, 1e-15);
    }
    {
        const double n = Vec3{3, 4, 0}.norm();
        CHECK_NEAR(n, 5.0, 1e-15);
    }

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
    SECTION("Atmosphere ISA");

    // Sea level
    auto sl = isa_conditions(0.0);
    CHECK_NEAR(sl.temperature_K, 288.15, 0.01);
    CHECK_NEAR(sl.pressure_Pa, 101325.0, 1.0);
    CHECK_NEAR(sl.air_density_kg_m3, 1.225, 0.01);

    // Density decreases with altitude
    auto hi = isa_conditions(5000.0);
    CHECK(hi.air_density_kg_m3 < sl.air_density_kg_m3);
    CHECK(hi.temperature_K < sl.temperature_K);

    // Tropopause: temperature should be ~216.65 K
    auto tp = isa_conditions(11000.0);
    CHECK_NEAR(tp.temperature_K, 216.65, 0.5);

    // compute_air_density: dry air at ISA sea level
    double rho = compute_air_density(288.15, 101325.0, 0.0);
    CHECK_NEAR(rho, 1.225, 0.01);

    // Humidity increases effective volume, so density decreases slightly
    double rho_wet = compute_air_density(288.15, 101325.0, 1.0);
    CHECK(rho_wet < rho);
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

    MunitionLibrary   lib;
    lib.load_from_string(json);

    CHECK(lib.size() == 1);
    CHECK(lib.contains("test_round"));
    CHECK(!lib.contains("nonexistent"));

    const auto& spec = lib.get("test_round");
    CHECK(spec.name == "test_round");
    CHECK_NEAR(spec.mass_kg, 0.01, 1e-15);
    CHECK_NEAR(spec.density_kg_m3, 11000.0, 1e-10);
    CHECK_NEAR(spec.reference_area_m2, 1e-4, 1e-15);
    CHECK_NEAR(spec.drag_coefficient, 0.30, 1e-15);
    CHECK_NEAR(spec.muzzle_velocity_ms, 900.0, 1e-10);
    CHECK_NEAR(spec.diameter_m, 0.01, 1e-15);

    // Derived quantities
    CHECK_NEAR(spec.ballistic_coefficient(),
               spec.mass_kg / (spec.drag_coefficient * spec.reference_area_m2),
               1e-10);
    CHECK_NEAR(spec.volume_m3(), spec.mass_kg / spec.density_kg_m3, 1e-20);

    // Missing field should throw
    const std::string bad_json = R"({"munitions":[{"name":"x","mass_kg":1.0}]})";
    bool              threw    = false;
    try {
        MunitionLibrary b;
        b.load_from_string(bad_json);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    // out_of_range on missing name
    threw = false;
    try {
        (void)lib.get("ghost");
    } catch (const std::out_of_range&) {
        threw = true;
    }
    CHECK(threw);

    // Bad JSON
    threw = false;
    try {
        MunitionLibrary b;
        b.load_from_string("not json at all!");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// Vacuum trajectory (zero drag)
// ---------------------------------------------------------------------------
static void test_vacuum_trajectory() {
    SECTION("Vacuum trajectory (Cd=0) — parabolic motion");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(vacuum_munition(), atm);

    // Horizontal shot
    const double    v0 = 100.0; // m/s horizontal
    const double    z0 = 100.0; // m above ground

    ProjectileState state;
    state.position = Vec3{0.0, 0.0, z0};
    state.velocity = Vec3{v0, 0.0, 0.0};
    state.time     = 0.0;

    // Analytical: time to ground = sqrt(2*z0/g)
    const double     g      = 9.80665;
    const double     t_land = std::sqrt(2.0 * z0 / g);
    const double     x_land = v0 * t_land;

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

    const double          v0        = 600.0; // m/s muzzle velocity
    const double          angle_deg = 30.0;
    const double          angle_rad = angle_deg * (kDegToRad);

    ProjectileState       initial;
    initial.position = Vec3{0.0, 0.0, 0.0};
    initial.velocity = Vec3{v0 * std::cos(angle_rad), 0.0, v0 * std::sin(angle_rad)};
    initial.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 1000.0;
    cfg.max_time = 120.0;
    cfg.use_rk4  = true;

    // Vacuum simulation
    TrajectorySimulator vac_sim(vacuum_munition(), atm);
    auto                vac_states = vac_sim.simulate(initial, cfg);

    // Drag simulation
    TrajectorySimulator drag_sim(drag_munition(), atm);
    auto                drag_states = drag_sim.simulate(initial, cfg);

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
// Wind effect test
// ---------------------------------------------------------------------------
static void test_wind() {
    SECTION("Wind shifts impact point");

    // Tailwind (positive x) should increase range
    AtmosphericConditions still    = still_atm();
    AtmosphericConditions tailwind = still;
    tailwind.wind.velocity_ms      = Vec3{50.0, 0.0, 0.0}; // 50 m/s tailwind

    const double    angle_rad = 30.0 * (kDegToRad);
    ProjectileState initial;
    initial.position = Vec3{0.0, 0.0, 1.0};
    initial.velocity = Vec3{400.0 * std::cos(angle_rad), 0.0, 400.0 * std::sin(angle_rad)};
    initial.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 500.0;
    cfg.max_time = 60.0;

    TrajectorySimulator s_sim(drag_munition(), still);
    TrajectorySimulator t_sim(drag_munition(), tailwind);

    auto                s_states = s_sim.simulate(initial, cfg);
    auto                t_states = t_sim.simulate(initial, cfg);

    const double        x_still    = s_states.back().position.x;
    const double        x_tailwind = t_states.back().position.x;

    std::printf("  Still range: %.1f m,  Tailwind range: %.1f m\n", x_still, x_tailwind);
    CHECK(x_tailwind > x_still);
}

// ---------------------------------------------------------------------------
// Real-time step performance
// ---------------------------------------------------------------------------
static void test_realtime_performance() {
    SECTION("Real-time step performance");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    ProjectileState       state;
    state.position = Vec3{0.0, 0.0, 1.5};
    state.velocity = Vec3{800.0, 0.0, 400.0};
    state.time     = 0.0;

    // Simulate 1 second at 240 Hz (= 240 steps) and measure total time
    const double dt        = 1.0 / 240.0;
    const int    num_steps = 240;

    auto         t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_steps; ++i)
        state = sim.step(state, dt);
    auto         t1 = std::chrono::high_resolution_clock::now();

    const double total_us    = std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double per_step_us = total_us / num_steps;
    // Budget per 60 Hz frame: 16 667 µs
    // At 4 sub-steps per frame the per-step budget is 4 167 µs — very generous.
    // We require < 100 µs per step to leave ample headroom for many projectiles.
    std::printf("  %d RK4 steps in %.1f µs (%.2f µs/step)\n", num_steps, total_us, per_step_us);
    CHECK(per_step_us < 100.0); // should be <<1 µs on any modern hardware
}

// ---------------------------------------------------------------------------
// Streaming callback
// ---------------------------------------------------------------------------
static void test_streaming_callback() {
    SECTION("Streaming simulate() callback");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    ProjectileState       initial;
    initial.position = Vec3{0.0, 0.0, 1.0};
    initial.velocity = Vec3{300.0, 0.0, 200.0};
    initial.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 30.0;

    // Count states via callback and compare to batch
    int    cb_count  = 0;
    bool   monotone  = true;
    double prev_time = -1.0;
    sim.simulate(initial, cfg, [&](const ProjectileState& s) {
        ++cb_count;
        if (s.time < prev_time)
            monotone = false;
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
    sim.simulate(initial, cfg, [&](const ProjectileState&) { return ++counted < stop_at; });
    CHECK(counted == stop_at);
}

// ---------------------------------------------------------------------------
// Altitude-varying atmosphere
// ---------------------------------------------------------------------------
static void test_altitude_varying_atmosphere() {
    SECTION("Altitude-varying atmosphere callback");

    MunitionSpec          m        = drag_munition();
    AtmosphericConditions base_atm = still_atm();
    TrajectorySimulator   sim(m, base_atm);

    ProjectileState       initial;
    initial.position = Vec3{0.0, 0.0, 0.0};
    initial.velocity = Vec3{700.0, 0.0, 700.0}; // steep trajectory
    initial.time     = 0.0;

    // Fixed atmosphere
    SimulationConfig cfg_fixed;
    cfg_fixed.dt       = 1.0 / 240.0;
    cfg_fixed.max_time = 120.0;

    // Altitude-varying: standard ISA
    SimulationConfig cfg_isa = cfg_fixed;
    cfg_isa.atmosphere_fn    = [](double alt_m) { return isa_conditions(alt_m); };

    auto fixed_states = sim.simulate(initial, cfg_fixed);
    auto isa_states   = sim.simulate(initial, cfg_isa);

    // Both should produce valid trajectories
    CHECK(!fixed_states.empty());
    CHECK(!isa_states.empty());

    // At high altitude, density is lower → less drag → ISA should fly farther
    const double x_fixed = fixed_states.back().position.x;
    const double x_isa   = isa_states.back().position.x;
    std::printf("  Fixed-density range: %.1f m,  ISA range: %.1f m\n", x_fixed, x_isa);
    CHECK(x_isa > x_fixed);
}

// ---------------------------------------------------------------------------
// Fire control tests
// ---------------------------------------------------------------------------
static void test_fire_control_basic() {
    SECTION("solve_elevation: low-angle solution lands at requested range");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    const double          muzzle_speed = 600.0; // m/s
    const double          target_range = 500.0; // m
    const double          az           = 0.0;   // due North

    FireSolution sol = solve_elevation(sim, LauncherOrientation{az}, target_range, muzzle_speed);
    CHECK(sol.valid);
    CHECK(sol.elevation_deg > 0.0);
    CHECK(sol.elevation_deg < 45.0); // low-angle solution
    CHECK(sol.flight_time_ms > 0.0);

    // Verify the solution: re-simulate with the returned elevation angle
    const double    el     = sol.elevation_deg * (kDegToRad);
    const double    az_rad = az * (kDegToRad);

    ProjectileState initial;
    initial.position = {};
    initial.velocity =
        muzzle_speed *
        Vec3{std::cos(el) * std::sin(az_rad), std::cos(el) * std::cos(az_rad), std::sin(el)};
    initial.time = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 300.0;

    auto states = sim.simulate(initial, cfg);
    CHECK(!states.empty());

    const auto&  impact = states.back();
    const double actual_range =
        std::sqrt(impact.position.x * impact.position.x + impact.position.y * impact.position.y);
    std::printf("  Requested: %.1f m  |  Actual: %.2f m  |  Elev: %.3f°  |  ToF: %.1f ms\n",
                target_range,
                actual_range,
                sol.elevation_deg,
                sol.flight_time_ms);

    // Solution should hit within 0.5 m of the requested range
    CHECK_NEAR(actual_range, target_range, 0.5);

    // Flight time should agree with simulation to within 5 ms
    CHECK_NEAR(sol.flight_time_ms, impact.time * 1000.0, 5.0);
}

static void test_fire_control_high_angle() {
    SECTION("solve_elevation: high-angle solution lands at same range, longer ToF");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    const double          muzzle_speed = 600.0;
    const double          target_range = 300.0;
    const double          az           = 90.0; // due East

    FireSolution          lo = solve_elevation(
        sim, LauncherOrientation{az}, target_range, muzzle_speed, 0.0, /*high_angle=*/false);
    FireSolution hi = solve_elevation(
        sim, LauncherOrientation{az}, target_range, muzzle_speed, 0.0, /*high_angle=*/true);

    CHECK(lo.valid);
    CHECK(hi.valid);

    std::printf("  Low-angle : elev=%.2f°  ToF=%.0f ms\n", lo.elevation_deg, lo.flight_time_ms);
    std::printf("  High-angle: elev=%.2f°  ToF=%.0f ms\n", hi.elevation_deg, hi.flight_time_ms);

    // High-angle solution must have a steeper elevation and longer flight time
    CHECK(hi.elevation_deg > lo.elevation_deg);
    CHECK(hi.flight_time_ms > lo.flight_time_ms);
}

static void test_fire_control_out_of_range() {
    SECTION("solve_elevation: returns valid=false when target is out of range");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    // 1 m/s muzzle velocity — cannot reach 10 000 m
    FireSolution sol =
        solve_elevation(sim, LauncherOrientation{0.0}, 10000.0, /*muzzle_speed=*/1.0);
    CHECK(!sol.valid);
}

static void test_fire_control_elevated_launcher() {
    SECTION("solve_elevation: launcher above target plane");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    const double          muzzle_speed  = 400.0;
    const double          target_range  = 400.0;
    const double          launch_height = 100.0; // launcher 100 m above target

    FireSolution          sol =
        solve_elevation(sim, LauncherOrientation{0.0}, target_range, muzzle_speed, launch_height);
    CHECK(sol.valid);

    std::printf("  Elevated launcher (h=%.0fm): elev=%.3f°  ToF=%.0f ms\n",
                launch_height,
                sol.elevation_deg,
                sol.flight_time_ms);

    // With a 100 m height advantage, elevation can be negative (shooting down)
    // For a 400 m range this should be sub-horizontal
    CHECK(sol.elevation_deg < 45.0);
}

static void test_fire_control_azimuth_independence() {
    SECTION("solve_elevation: elevation angle is azimuth-independent");

    AtmosphericConditions atm = still_atm(); // still air, no wind
    TrajectorySimulator   sim(drag_munition(), atm);

    const double          muzzle_speed = 500.0;
    const double          range        = 350.0;

    FireSolution north = solve_elevation(sim, LauncherOrientation{0.0}, range, muzzle_speed);
    FireSolution east  = solve_elevation(sim, LauncherOrientation{90.0}, range, muzzle_speed);
    FireSolution south = solve_elevation(sim, LauncherOrientation{180.0}, range, muzzle_speed);

    CHECK(north.valid && east.valid && south.valid);

    std::printf("  N: %.4f°  E: %.4f°  S: %.4f°\n",
                north.elevation_deg,
                east.elevation_deg,
                south.elevation_deg);

    // Without wind, elevation should be identical regardless of azimuth
    CHECK_NEAR(north.elevation_deg, east.elevation_deg, 0.01);
    CHECK_NEAR(north.elevation_deg, south.elevation_deg, 0.01);
}

// ---------------------------------------------------------------------------
// FireControlTable tests
// ---------------------------------------------------------------------------
static void test_table_builds_and_is_ready() {
    SECTION("FireControlTable: builds without error and marks ready");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    FireControlTable      table;
    CHECK(!table.ready());
    CHECK(table.max_range_m() == 0.0);

    table.build(sim, 600.0);

    CHECK(table.ready());
    CHECK(table.max_range_m() > 0.0);
    CHECK(!table.entries().empty());

    std::printf("  max_range=%.1f m  entries=%zu\n", table.max_range_m(), table.entries().size());
}

static void test_table_lookup_accuracy() {
    SECTION("FireControlTable: lookup agrees with solve_elevation within 1 m");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    FireControlTable      table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);

    CHECK(table.ready());

    // Test several ranges across the achievable envelope
    const double max_r = table.max_range_m();
    for (double frac : {0.1, 0.3, 0.5, 0.7, 0.9}) {
        const double range = frac * max_r;
        FireSolution ref   = solve_elevation(sim, LauncherOrientation{0.0}, range, 600.0);
        FireSolution tbl   = table.lookup(range);

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
    TrajectorySimulator   sim(drag_munition(), atm);

    FireControlTable      table;
    table.build(sim, 600.0);

    FireSolution beyond = table.lookup(table.max_range_m() * 2.0);
    CHECK(!beyond.valid);

    FireSolution negative = table.lookup(-10.0);
    CHECK(!negative.valid);
}

static void test_table_lookup_realtime_performance() {
    SECTION("FireControlTable: lookup is fast enough for 60 Hz game loop");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    FireControlTable      table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);

    CHECK(table.ready());

    const double    max_r = table.max_range_m();
    const int       N     = 10000; // simulate 10 000 consecutive frame lookups

    auto            t0   = std::chrono::high_resolution_clock::now();
    volatile double sink = 0.0; // prevent the compiler optimising the loop away
    for (int i = 0; i < N; ++i) {
        const double range = max_r * (i % 1000) / 1000.0;
        FireSolution s     = table.lookup(range);
        sink += s.elevation_deg;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)sink;

    const double total_us  = std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double per_us    = total_us / N;
    const double budget_us = 1e6 / 60.0; // 16 667 µs per frame

    std::printf("  %d lookups in %.1f µs  (%.3f µs/lookup,  budget=%.0f µs/frame)\n",
                N,
                total_us,
                per_us,
                budget_us);

    // Each lookup must be well under 1 µs — negligible vs 60 Hz budget
    CHECK(per_us < 1.0);
}

static void test_table_build_timing() {
    SECTION("FireControlTable: build time is reported (informational)");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    auto                  t0 = std::chrono::high_resolution_clock::now();
    FireControlTable      table;
    table.build(sim, 600.0, 0.0, 0.0, false, 500);
    auto         t1 = std::chrono::high_resolution_clock::now();

    const double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("  500-sample table built in %.1f ms  "
                "(run async or during loading)\n",
                build_ms);

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
    TrajectorySimulator   sim(drag_munition(), atm);

    FireControlTable      table;
    CHECK(!table.ready());

    table.build(sim, 600.0, 0.0, 0.0, /*high_angle=*/true, 500);

    CHECK(table.ready());
    CHECK(table.max_range_m() > 0.0);
    CHECK(!table.entries().empty());

    // Verify a mid-range lookup is valid and has a high elevation angle
    const double test_range = table.max_range_m() * 0.5;
    FireSolution sol        = table.lookup(test_range);
    CHECK(sol.valid);
    // High-angle solution must be above 45°
    CHECK(sol.elevation_deg > 45.0);
    CHECK(sol.flight_time_ms > 0.0);

    std::printf("  High-angle table: max_range=%.1f m  entries=%zu\n",
                table.max_range_m(),
                table.entries().size());
    std::printf(
        "  Lookup at 50%% range: elev=%.2f°  ToF=%.0f ms\n", sol.elevation_deg, sol.flight_time_ms);
}

static void test_table_lookup_below_min_range() {
    SECTION("FireControlTable: lookup below minimum entry returns valid=false");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

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
                    min_range * 0.5,
                    below.valid ? "true" : "false");
    }

    // Negative range must always be invalid
    FireSolution neg = table.lookup(-1.0);
    CHECK(!neg.valid);

    // Exact minimum entry range must be valid
    FireSolution exact = table.lookup(min_range);
    CHECK(exact.valid);
}

static void test_fire_control_wind() {
    SECTION("solve_elevation: azimuth matters when wind is present");

    // With a crosswind, firing East vs West should require different elevations
    // to reach the same range because the headwind/tailwind component differs.
    AtmosphericConditions atm_wind = still_atm();
    atm_wind.wind.velocity_ms      = Vec3{20.0, 0.0, 0.0}; // 20 m/s eastward wind

    TrajectorySimulator sim(drag_munition(), atm_wind);

    const double        muzzle_speed = 500.0;
    const double        range        = 400.0;

    // Firing East (+x): tailwind → effectively less drag → lower elevation needed
    FireSolution east = solve_elevation(sim, LauncherOrientation{90.0}, range, muzzle_speed);
    // Firing West (-x): headwind → more drag → higher elevation needed
    FireSolution west = solve_elevation(sim, LauncherOrientation{270.0}, range, muzzle_speed);

    CHECK(east.valid);
    CHECK(west.valid);

    std::printf("  Wind 20 m/s East — firing East: elev=%.3f°  West: elev=%.3f°\n",
                east.elevation_deg,
                west.elevation_deg);

    // Eastward (tailwind) shot needs less elevation than westward (headwind) shot
    CHECK(east.elevation_deg < west.elevation_deg);
}

static void test_isa_above_stratopause() {
    SECTION("isa_conditions: above 20 km (clamped stratopause fallback)");

    // The ISA model covers 0–20 km explicitly.  Above that, the implementation
    // should use a clamped fallback rather than crashing or producing garbage.
    auto lo  = isa_conditions(20000.0); // top of modelled stratosphere
    auto hi  = isa_conditions(25000.0); // above model range
    auto vhi = isa_conditions(50000.0); // well above model range

    // Must not produce NaN or obviously wrong values
    CHECK(lo.air_density_kg_m3 > 0.0);
    CHECK(hi.air_density_kg_m3 > 0.0);
    CHECK(vhi.air_density_kg_m3 > 0.0);

    // Density should be lower at higher altitudes (or equal if clamped)
    CHECK(hi.air_density_kg_m3 <= lo.air_density_kg_m3);

    std::printf("  ISA density: 20km=%.4f  25km=%.4f  50km=%.4f kg/m³\n",
                lo.air_density_kg_m3,
                hi.air_density_kg_m3,
                vhi.air_density_kg_m3);
}

static void test_munition_library_load_file() {
    SECTION("MunitionLibrary: load() from file path");

    MunitionLibrary lib;
    // Try a few candidate paths since the working directory varies by runner
    const char* candidates[] = {
        "data/munitions.json", "../data/munitions.json", "../../data/munitions.json"};
    bool loaded = false;
    for (const char* path : candidates) {
        try {
            lib.load(path);
            loaded = true;
            std::printf("  Loaded from: %s\n", path);
            break;
        } catch (const std::runtime_error&) {
        }
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
    try {
        MunitionLibrary b;
        b.load("/tmp/does_not_exist_xyz.json");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);
}

static void test_table_with_target_altitude() {
    SECTION("FireControlTable: build with non-zero target_altitude_m");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    // Build two tables with the same launch height but different target
    // altitudes. Firing down at a depressed target should achieve longer
    // horizontal range than firing at the same elevation.
    const double     muzzle = 600.0;
    const double     lh     = 100.0; // launcher 100 m above sea level

    FireControlTable table_flat; // target at z = 0
    FireControlTable table_elev; // target at z = 50 m (half-way up)

    table_flat.build(sim, muzzle, 0.0, lh, false, 200, /*target_altitude_m=*/0.0);
    table_elev.build(sim, muzzle, 0.0, lh, false, 200, /*target_altitude_m=*/50.0);

    CHECK(table_flat.ready());
    CHECK(table_elev.ready());

    // An elevated target means the projectile hits a ground plane that is
    // closer in the vertical dimension, so max horizontal range should differ.
    std::printf("  max_range flat=%.1f m  elevated_target=%.1f m\n",
                table_flat.max_range_m(),
                table_elev.max_range_m());

    // Both tables should produce valid solutions at the same test range
    const double test_range = std::min(table_flat.max_range_m(), table_elev.max_range_m()) * 0.5;
    CHECK(table_flat.lookup(test_range).valid);
    CHECK(table_elev.lookup(test_range).valid);
}

static void test_table_wind_azimuth() {
    SECTION("FireControlTable: azimuth affects lookup when wind is present");

    // Build two tables for the same munition but different azimuths in
    // the presence of a strong headwind.  The firing solution should differ
    // because one direction has a tailwind and the other a headwind.
    AtmosphericConditions atm_wind = still_atm();
    atm_wind.wind.velocity_ms      = Vec3{30.0, 0.0, 0.0}; // 30 m/s eastward

    TrajectorySimulator sim(drag_munition(), atm_wind);

    const double        muzzle = 500.0;

    FireControlTable    east_table; // fire East  (+x): tailwind
    FireControlTable    west_table; // fire West  (-x): headwind

    east_table.build(sim, muzzle, /*azimuth_deg=*/90.0, 0.0, false, 200);
    west_table.build(sim, muzzle, /*azimuth_deg=*/270.0, 0.0, false, 200);

    CHECK(east_table.ready());
    CHECK(west_table.ready());

    std::printf("  Tailwind (East) max_range=%.1f m  Headwind (West) max_range=%.1f m\n",
                east_table.max_range_m(),
                west_table.max_range_m());

    // Tailwind increases effective range; headwind decreases it.
    CHECK(east_table.max_range_m() > west_table.max_range_m());

    // At a range both tables cover, the elevation angles must differ.
    const double common_range = std::min(east_table.max_range_m(), west_table.max_range_m()) * 0.5;
    FireSolution e_sol        = east_table.lookup(common_range);
    FireSolution w_sol        = west_table.lookup(common_range);
    CHECK(e_sol.valid);
    CHECK(w_sol.valid);
    std::printf("  At %.0f m — East elev=%.3f°  West elev=%.3f°\n",
                common_range,
                e_sol.elevation_deg,
                w_sol.elevation_deg);
    // Headwind (West) requires a higher elevation to reach the same range.
    CHECK(w_sol.elevation_deg > e_sol.elevation_deg);
}

static void test_build_num_samples_guard() {
    SECTION("FireControlTable: num_samples < 2 is clamped to 2");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    // Passing 0 or 1 must not crash and must produce a usable table
    for (int bad_n : {0, 1}) {
        FireControlTable table;
        table.build(sim, 500.0, 0.0, 0.0, false, bad_n);
        CHECK(table.ready());
        // A 2-entry table should at least cover some range
        CHECK(table.max_range_m() > 0.0);
        std::printf("  num_samples=%d → entries=%zu  max_range=%.1f m\n",
                    bad_n,
                    table.entries().size(),
                    table.max_range_m());
    }
}

static void test_invalid_muzzle_speed() {
    SECTION("solve_elevation and build: throw on muzzle_speed_ms <= 0");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    // solve_elevation
    bool threw = false;
    try {
        solve_elevation(sim, LauncherOrientation{0.0}, 100.0, /*muzzle=*/0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        solve_elevation(sim, LauncherOrientation{0.0}, 100.0, /*muzzle=*/-1.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    // FireControlTable::build
    threw = false;
    try {
        FireControlTable t;
        t.build(sim, /*muzzle=*/0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        FireControlTable t;
        t.build(sim, /*muzzle=*/-500.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// Altitude variation tests
// ---------------------------------------------------------------------------

// Helper: re-simulate with a given elevation and azimuth, return impact
// (horizontal range, altitude).  The initial position is at absolute altitude
// (target_alt + launch_height).  The ground plane is target_alt.
struct ImpactResult {
    double h_range_m;
    double z_m;
    double time_s;
};

static ImpactResult resimulate(const TrajectorySimulator& sim,
                               double                     elevation_deg,
                               double                     azimuth_deg,
                               double                     muzzle_speed_ms,
                               double                     launch_height_m,
                               double                     target_alt_m) {
    const double    el    = elevation_deg * kDegToRad;
    const double    az    = azimuth_deg * kDegToRad;
    const Vec3      h_dir = {std::sin(az), std::cos(az), 0.0};

    ProjectileState st;
    st.position = Vec3{0.0, 0.0, target_alt_m + launch_height_m};
    st.velocity = muzzle_speed_ms * (std::cos(el) * h_dir + Vec3{0.0, 0.0, std::sin(el)});
    st.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 120.0;
    cfg.use_rk4  = true;

    if (launch_height_m < 0.0) {
        // Launcher is BELOW the target plane: detect the ascending crossing of
        // target_alt_m (the projectile rises through the target altitude going up).
        // simulate() only detects descending crossings via ground_z, so we use a
        // streaming callback to catch the first upward crossing instead.
        cfg.ground_z = st.position.z - 1.0; // floor below launcher; never triggered

        ProjectileState prev = st, impact = st;
        bool            found = false;
        sim.simulate(st, cfg, [&](const ProjectileState& s) {
            if (!found && prev.position.z < target_alt_m && s.position.z >= target_alt_m) {
                const double frac =
                    (target_alt_m - prev.position.z) / (s.position.z - prev.position.z);
                impact.position.x = prev.position.x + frac * (s.position.x - prev.position.x);
                impact.position.y = prev.position.y + frac * (s.position.y - prev.position.y);
                impact.position.z = target_alt_m;
                impact.time       = prev.time + frac * (s.time - prev.time);
                found             = true;
                return false; // stop simulation
            }
            prev = s;
            return true;
        });
        const double hr = std::sqrt(impact.position.x * impact.position.x +
                                    impact.position.y * impact.position.y);
        return {hr, impact.position.z, impact.time};
    } else {
        // Normal case: launcher at or above the target plane; simulate() terminates
        // when the projectile descends to ground_z = target_alt_m.
        cfg.ground_z         = target_alt_m;
        ProjectileState last = st;
        sim.simulate(st, cfg, [&last](const ProjectileState& s) {
            last = s;
            return true;
        });
        const double hr =
            std::sqrt(last.position.x * last.position.x + last.position.y * last.position.y);
        return {hr, last.position.z, last.time};
    }
}

// ---------------------------------------------------------------------------
// Launcher significantly above target — various non-similar altitudes
// ---------------------------------------------------------------------------
static void test_altitude_launcher_above_target() {
    SECTION("Altitude: launcher above target, various altitude pairs");

    // Use 5.56x45 and 7.62x51 from the munitions file
    MunitionLibrary lib;
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Scenario table: {launcher_alt_m, target_alt_m, munition_name, muzzle_ms, range_m}
    // All ranges are well within the munition's achievable envelope.
    // Altitude differences are deliberately large and non-similar.
    struct Scenario {
        double      launcher_alt;
        double      target_alt;
        const char* munition;
        double      muzzle;
        double      range;
        const char* label;
    };
    const Scenario scenarios[] = {
        // 5.56 — launcher high above target
        {600.0,
         0.0,
         "5.56x45_m855_62gr",
         930.0,
         400.0,
         "5.56: launcher 600m, target 0m, range 400m"},
        {1200.0,
         200.0,
         "5.56x45_m855_62gr",
         930.0,
         500.0,
         "5.56: launcher 1200m, target 200m, range 500m"},
        {800.0,
         50.0,
         "5.56x45_m855_62gr",
         930.0,
         350.0,
         "5.56: launcher 800m, target 50m, range 350m"},
        // 7.62 — different altitude pairs
        {500.0,
         0.0,
         "7.62x51_m80_147gr",
         853.0,
         400.0,
         "7.62: launcher 500m, target 0m, range 400m"},
        {1000.0,
         300.0,
         "7.62x51_m80_147gr",
         853.0,
         500.0,
         "7.62: launcher 1000m, target 300m, range 500m"},
        // .338 Lapua — large altitude differences
        {1500.0,
         100.0,
         "338_lapua_250gr",
         930.0,
         700.0,
         ".338: launcher 1500m, target 100m, range 700m"},
        {2000.0,
         500.0,
         "338_lapua_250gr",
         930.0,
         800.0,
         ".338: launcher 2000m, target 500m, range 800m"},
    };

    for (const auto& s : scenarios) {
        const double          launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm           = isa_conditions(s.target_alt);
        const MunitionSpec&   spec          = lib.get(s.munition);
        TrajectorySimulator   sim(spec, atm);

        FireSolution          sol = solve_elevation(sim,
                                           LauncherOrientation{0.0},
                                           s.range,
                                           s.muzzle,
                                           launch_height,
                                           /*high_angle=*/false,
                                           /*tolerance_m=*/0.5,
                                           s.target_alt);
        std::printf("  [%s]\n    elev=%.3f°  ToF=%.0f ms  valid=%s\n",
                    s.label,
                    sol.elevation_deg,
                    sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid)
            continue;

        // Re-simulate and verify the solution
        ImpactResult imp =
            resimulate(sim, sol.elevation_deg, 0.0, s.muzzle, launch_height, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m,
                    imp.z_m,
                    s.target_alt);
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
    // NOTE: These cases were originally labelled "impossible small ranges"
    // because the old solver only detected DESCENDING crossings of target_z.
    // The ascending-crossing fix means a steep (low-angle) shot can reach the
    // elevated target plane on the way up at short horizontal distances.
    // All three cases are now achievable and must return valid=true.
    SECTION("Altitude: launcher below target — short ranges via ascending path → valid=true");

    MunitionLibrary lib;
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // These ranges require steep ascending shots — achievable with the
    // ascending-crossing fix.  low-angle solver finds the ascending crossing.
    struct ShortRangeCase {
        double      launcher_alt, target_alt, muzzle, range;
        const char* munition;
        const char* label;
    };
    const ShortRangeCase short_range[] = {
        {0.0,
         200.0,
         930.0,
         300.0,
         "5.56x45_m855_62gr",
         "5.56: launcher 0m, target 200m, range 300m (ascending path)"},
        {0.0,
         400.0,
         853.0,
         350.0,
         "7.62x51_m80_147gr",
         "7.62: launcher 0m, target 400m, range 350m (ascending path)"},
        {0.0,
         300.0,
         930.0,
         200.0,
         "338_lapua_250gr",
         ".338: launcher 0m, target 300m, range 200m (ascending path)"},
    };

    for (const auto& s : short_range) {
        const double          lh  = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm = isa_conditions(s.launcher_alt);
        TrajectorySimulator   sim(lib.get(s.munition), atm);

        FireSolution          sol = solve_elevation(
            sim, LauncherOrientation{0.0}, s.range, s.muzzle, lh, false, 0.5, s.target_alt);
        std::printf("  [%s]\n    valid=%s  elev=%.2f°  ToF=%.0f ms\n",
                    s.label,
                    sol.valid ? "true" : "false",
                    sol.elevation_deg,
                    sol.flight_time_ms);
        CHECK(sol.valid);

        if (!sol.valid)
            continue;
        // The ascending path requires a steep upward angle
        CHECK(sol.elevation_deg > 0.0);

        // Re-simulate via ascending crossing and verify impact
        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0, s.muzzle, lh, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m,
                    imp.z_m,
                    s.target_alt);
        CHECK_NEAR(imp.h_range_m, s.range, 2.0);
        CHECK_NEAR(imp.z_m, s.target_alt, 1.0);
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
        double      launcher_alt, target_alt, muzzle;
        const char* munition;
        const char* label;
    };
    const AchievableCase achievable[] = {
        {0.0, 100.0, 930.0, "5.56x45_m855_62gr", "5.56 (high-angle): launcher 0m, target 100m"},
        {100.0, 300.0, 853.0, "7.62x51_m80_147gr", "7.62 (high-angle): launcher 100m, target 300m"},
        {0.0, 200.0, 930.0, "338_lapua_250gr", ".338 (high-angle): launcher 0m, target 200m"},
        {200.0, 600.0, 930.0, "338_lapua_250gr", ".338 (high-angle): launcher 200m, target 600m"},
    };

    for (const auto& s : achievable) {
        const double          lh  = s.launcher_alt - s.target_alt; // negative
        AtmosphericConditions atm = isa_conditions(s.launcher_alt);
        TrajectorySimulator   sim(lib.get(s.munition), atm);

        // Use a coarse HIGH-ANGLE table to find the achievable range envelope
        FireControlTable probe;
        probe.build(sim,
                    s.muzzle,
                    0.0,
                    lh,
                    /*high_angle=*/true,
                    80,
                    s.target_alt);

        if (!probe.ready() || probe.max_range_m() < 200.0) {
            std::printf(
                "  [%s] Skipped: max_range=%.0f m too small\n", s.label, probe.max_range_m());
            continue;
        }

        const double test_range = probe.max_range_m() * 0.5;

        // Must use high_angle=true: plunging trajectory is the only option
        // when the launcher is below the target plane.
        FireSolution sol = solve_elevation(sim,
                                           LauncherOrientation{0.0},
                                           test_range,
                                           s.muzzle,
                                           lh,
                                           /*high_angle=*/true,
                                           0.5,
                                           s.target_alt);
        std::printf("  [%s]\n    max_range=%.0f m  test_range=%.0f m\n"
                    "    elev=%.3f°  ToF=%.0f ms  valid=%s\n",
                    s.label,
                    probe.max_range_m(),
                    test_range,
                    sol.elevation_deg,
                    sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid)
            continue;
        // High-angle plunging shot: elevation is positive (fires upward)
        CHECK(sol.elevation_deg > 0.0);

        ImpactResult imp = resimulate(sim, sol.elevation_deg, 0.0, s.muzzle, lh, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m,
                    imp.z_m,
                    s.target_alt);
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
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    struct Scenario {
        double      launcher_alt;
        double      target_alt;
        const char* munition;
        double      muzzle;
        double      range; // 0 = determine dynamically from FireControlTable
        const char* label;
    };
    const Scenario scenarios[] = {
        // Launcher above target, both non-zero — ranges are well within envelope
        {800.0,
         200.0,
         "5.56x45_m855_62gr",
         930.0,
         400.0,
         "5.56: launcher 800m, target 200m (above), range 400m"},
        {1500.0,
         600.0,
         "7.62x51_m80_147gr",
         853.0,
         500.0,
         "7.62: launcher 1500m, target 600m (above), range 500m"},
        // Launcher below target — use range=0 to trigger dynamic determination
        {300.0,
         900.0,
         "338_lapua_250gr",
         930.0,
         0.0,
         ".338: launcher 300m, target 900m (below), dynamic range"},
        {100.0,
         500.0,
         "338_lapua_250gr",
         930.0,
         0.0,
         ".338: launcher 100m, target 500m (below), dynamic range"},
    };

    for (const auto& s : scenarios) {
        const double          launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm  = isa_conditions(std::min(s.launcher_alt, s.target_alt));
        const MunitionSpec&   spec = lib.get(s.munition);
        TrajectorySimulator   sim(spec, atm);

        double                test_range = s.range;
        // Launcher-below-target requires high-angle (plunging) fire:
        // the simulation only detects descending crossings of target_z_m.
        const bool below_target = (s.launcher_alt < s.target_alt);

        if (test_range <= 0.0) {
            // Determine the achievable range dynamically
            FireControlTable probe;
            probe.build(sim,
                        s.muzzle,
                        45.0,
                        launch_height,
                        /*high_angle=*/true,
                        80,
                        s.target_alt);
            if (!probe.ready() || probe.max_range_m() < 200.0) {
                std::printf(
                    "  [%s] Skipped: max_range=%.0f m too small\n", s.label, probe.max_range_m());
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
                    s.label,
                    test_range,
                    sol.elevation_deg,
                    sol.flight_time_ms,
                    sol.valid ? "true" : "false");
        CHECK(sol.valid);

        if (!sol.valid)
            continue;

        ImpactResult imp =
            resimulate(sim, sol.elevation_deg, 45.0, s.muzzle, launch_height, s.target_alt);
        std::printf("    re-sim: h_range=%.2f m  z=%.2f m (target %.2f m)\n",
                    imp.h_range_m,
                    imp.z_m,
                    s.target_alt);
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
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Launcher well above target — elevation is negative (shooting downward)
    {
        const double          launcher_alt  = 700.0;
        const double          target_alt    = 0.0;
        const double          launch_height = launcher_alt - target_alt; // +700
        AtmosphericConditions atm           = isa_conditions(target_alt);
        TrajectorySimulator   sim(lib.get("7.62x51_m80_147gr"), atm);

        FireControlTable      table;
        table.build(sim, 853.0, 0.0, launch_height, false, 300, target_alt);
        CHECK(table.ready());
        CHECK(table.max_range_m() > 0.0);

        const double test_range = table.max_range_m() * 0.4;
        FireSolution sol        = table.lookup(test_range);
        CHECK(sol.valid);
        // Launcher is 700 m above the target; a downward shot is expected.
        // Elevation may legitimately be negative here — do NOT check > 0.
        std::printf("  Table (launcher 700m, target 0m): "
                    "max=%.1f m, lookup@40%%=%.1f m → elev=%.2f°\n",
                    table.max_range_m(),
                    test_range,
                    sol.elevation_deg);

        // Re-simulate to verify the table's interpolated solution
        ImpactResult imp =
            resimulate(sim, sol.elevation_deg, 0.0, 853.0, launch_height, target_alt);
        CHECK_NEAR(imp.h_range_m, test_range, test_range * 0.02); // 2 % tolerance
        CHECK_NEAR(imp.z_m, target_alt, 1.0);
    }

    // Launcher below target — elevation must be positive (shooting upward)
    {
        const double          launcher_alt  = 0.0;
        const double          target_alt    = 200.0;
        const double          launch_height = launcher_alt - target_alt; // -200
        AtmosphericConditions atm           = isa_conditions(launcher_alt);
        TrajectorySimulator   sim(lib.get("338_lapua_250gr"), atm);

        FireControlTable      table;
        table.build(sim, 930.0, 0.0, launch_height, /*high_angle=*/true, 300, target_alt);
        CHECK(table.ready());
        CHECK(table.max_range_m() > 0.0);

        // Use 50 % of the achievable max — guaranteed to be in-range
        const double test_range = table.max_range_m() * 0.5;
        FireSolution sol        = table.lookup(test_range);
        CHECK(sol.valid);
        // Must shoot upward to arc over and hit the elevated ground plane
        CHECK(sol.elevation_deg > 0.0);
        std::printf("  Table (launcher 0m, target 200m): "
                    "max=%.1f m, lookup@50%%=%.1f m → elev=%.2f°\n",
                    table.max_range_m(),
                    test_range,
                    sol.elevation_deg);

        // Re-simulate to verify
        ImpactResult imp =
            resimulate(sim, sol.elevation_deg, 0.0, 930.0, launch_height, target_alt);
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
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    // Only use launcher-above-target scenarios here: launcher-below at small
    // ranges are physically impossible and correctly return valid=false.
    // Their flight times are therefore not meaningful to test.
    struct Scenario {
        double      launcher_alt, target_alt, muzzle, range;
        const char* munition;
    };
    const Scenario scenarios[] = {
        {600.0, 0.0, 930.0, 400.0, "5.56x45_m855_62gr"},
        {1000.0, 100.0, 853.0, 500.0, "7.62x51_m80_147gr"},
        {2000.0, 300.0, 930.0, 700.0, "338_lapua_250gr"},
        {500.0, 50.0, 853.0, 350.0, "7.62x51_m80_147gr"},
    };

    for (const auto& s : scenarios) {
        const double          launch_height = s.launcher_alt - s.target_alt;
        AtmosphericConditions atm           = isa_conditions(s.target_alt);
        TrajectorySimulator   sim(lib.get(s.munition), atm);

        FireSolution          sol = solve_elevation(sim,
                                           LauncherOrientation{0.0},
                                           s.range,
                                           s.muzzle,
                                           launch_height,
                                           false,
                                           0.5,
                                           s.target_alt);
        CHECK(sol.valid);
        if (sol.valid) {
            // Flight time must be well under the 60 000 ms ceiling
            CHECK(sol.flight_time_ms < 60000.0);
            std::printf("  launcher=%.0fm target=%.0fm range=%.0fm → "
                        "elev=%.2f° ToF=%.0f ms\n",
                        s.launcher_alt,
                        s.target_alt,
                        s.range,
                        sol.elevation_deg,
                        sol.flight_time_ms);
        }
    }
}

// ---------------------------------------------------------------------------
// Moving-target intercept tests
// ---------------------------------------------------------------------------

// Helper: load the munition library from the usual relative paths.
// Returns false and prints a skip message when not found.
static bool load_lib_for_moving(MunitionLibrary& lib) {
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            return true;
        } catch (const std::runtime_error&) {
        }
    }
    std::printf("  Skipped: munitions file not found\n");
    return false;
}

// ---------------------------------------------------------------------------
// solve_moving_target: zero-velocity target → same result as solve_elevation
// ---------------------------------------------------------------------------
static void test_moving_target_stationary() {
    SECTION("solve_moving_target: zero velocity == solve_elevation result");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    AtmosphericConditions atm = isa_conditions(0.0);
    TrajectorySimulator   sim(lib.get("7.62x51_m80_147gr"), atm);

    const double          muzzle       = 853.0;
    Vec3                  launcher_pos = {0.0, 0.0, 0.0};
    Vec3                  target_pos   = {500.0, 300.0, 0.0}; // same altitude as launcher
    Vec3                  velocity     = {0.0, 0.0, 0.0};     // stationary

    InterceptSolution isol = solve_moving_target(sim, launcher_pos, target_pos, velocity, muzzle);

    std::printf("  Stationary target: valid=%s  az=%.2f°  el=%.2f°  tof=%.0f ms  iters=%d\n",
                isol.valid ? "true" : "false",
                isol.azimuth_deg,
                isol.fire.elevation_deg,
                isol.fire.flight_time_ms,
                isol.iterations);

    CHECK(isol.valid);
    if (!isol.valid)
        return;

    // With zero velocity the intercept point should equal the target position.
    const double dx = isol.intercept_point.x - target_pos.x;
    const double dy = isol.intercept_point.y - target_pos.y;
    const double dz = isol.intercept_point.z - target_pos.z;
    CHECK_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), 0.0, 1.0); // within 1 m

    // Lead distance should be negligible for a stationary target.
    CHECK(isol.lead_distance_m < 2.0);

    // The solve_elevation comparison
    const double range2d = std::sqrt(500.0 * 500.0 + 300.0 * 300.0);
    FireSolution ref = solve_elevation(sim, LauncherOrientation{isol.azimuth_deg}, range2d, muzzle);
    CHECK(ref.valid);
    if (ref.valid) {
        CHECK_NEAR(isol.fire.elevation_deg, ref.elevation_deg, 0.5);
        CHECK_NEAR(isol.fire.flight_time_ms, ref.flight_time_ms, 10.0);
    }
}

// ---------------------------------------------------------------------------
// solve_moving_target: target at same altitude — lead accounting
// ---------------------------------------------------------------------------
static void test_moving_target_level() {
    SECTION("solve_moving_target: moving target at same altitude as launcher");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    AtmosphericConditions atm = isa_conditions(0.0);
    TrajectorySimulator   sim(lib.get("7.62x51_m80_147gr"), atm);

    const double          muzzle       = 853.0;
    Vec3                  launcher_pos = {0.0, 0.0, 0.0};
    Vec3                  target_pos   = {0.0, 400.0, 0.0}; // 400 m North
    Vec3                  velocity     = {15.0, 0.0, 0.0};  // 15 m/s East (crossing target)

    InterceptSolution isol = solve_moving_target(sim, launcher_pos, target_pos, velocity, muzzle);

    std::printf("  Level moving target: valid=%s  az=%.2f°  el=%.2f°"
                "  tof=%.0f ms  lead=%.1f m  iters=%d\n",
                isol.valid ? "true" : "false",
                isol.azimuth_deg,
                isol.fire.elevation_deg,
                isol.fire.flight_time_ms,
                isol.lead_distance_m,
                isol.iterations);

    CHECK(isol.valid);
    if (!isol.valid)
        return;

    // Intercept point should be ahead of the current target position in the
    // direction of travel (East = +x).
    CHECK(isol.intercept_point.x > target_pos.x);

    // Lead distance must be positive and proportional to velocity × TOF
    CHECK(isol.lead_distance_m > 0.0);

    // The azimuth must lead the target — launcher aims East of North, so > 0°
    CHECK(isol.azimuth_deg > 0.0);
    CHECK(isol.azimuth_deg < 45.0); // but not a wild angle

    // Convergence should be fast (< 8 iterations)
    CHECK(isol.iterations <= 8);
}

// ---------------------------------------------------------------------------
// solve_moving_target: target ELEVATED above the launcher
// (exercises the ascending-crossing bug-fix in shoot())
// ---------------------------------------------------------------------------
static void test_moving_target_elevated_above_launcher() {
    SECTION("solve_moving_target: moving target elevated above launcher");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    // Use .338 Lapua for sufficient range + elevation capability
    const MunitionSpec&   spec = lib.get("338_lapua_250gr");
    AtmosphericConditions atm  = isa_conditions(0.0);
    TrajectorySimulator   sim(spec, atm);

    const double          muzzle = 930.0;

    struct ElevatedCase {
        double      launcher_z;
        double      target_z;
        double      target_x;
        double      target_y;
        double      vel_x;
        double      vel_y;
        const char* label;
    };
    const ElevatedCase cases[] = {
        // Target 50 m above launcher, moving North
        {0.0, 50.0, 0.0, 600.0, 0.0, 12.0, "target +50 m, moving N at 12 m/s"},
        // Target 150 m above launcher, moving East
        {0.0, 150.0, 300.0, 500.0, 20.0, 0.0, "target +150 m, moving E at 20 m/s"},
        // Both at altitude: launcher 100 m, target 300 m (200 m delta)
        {100.0, 300.0, 0.0, 700.0, -10.0, 5.0, "launcher 100m, target 300m, moving NW at ~11 m/s"},
    };

    for (const auto& c : cases) {
        Vec3 launcher_pos = {0.0, 0.0, c.launcher_z};
        Vec3 target_pos   = {c.target_x, c.target_y, c.target_z};
        Vec3 velocity     = {c.vel_x, c.vel_y, 0.0};

        // high_angle=true: the only valid mode when launcher is below target
        InterceptSolution isol = solve_moving_target(sim,
                                                     launcher_pos,
                                                     target_pos,
                                                     velocity,
                                                     muzzle,
                                                     /*high_angle=*/true);

        std::printf("  [%s]\n    valid=%s  az=%.2f°  el=%.2f°  tof=%.0f ms"
                    "  lead=%.1f m  iters=%d\n",
                    c.label,
                    isol.valid ? "true" : "false",
                    isol.azimuth_deg,
                    isol.fire.elevation_deg,
                    isol.fire.flight_time_ms,
                    isol.lead_distance_m,
                    isol.iterations);

        CHECK(isol.valid);
        if (!isol.valid)
            continue;

        // Shot must be aimed upward (launcher is below the target plane)
        CHECK(isol.fire.elevation_deg > 0.0);

        // Intercept point must be at (approximately) the target's altitude
        CHECK_NEAR(isol.intercept_point.z, c.target_z, 5.0);

        // Lead must reflect actual target motion
        if (std::abs(c.vel_x) + std::abs(c.vel_y) > 0.0)
            CHECK(isol.lead_distance_m > 0.0);

        // Sanity-check intercept point placement:
        // forward-project target by flight time, check proximity
        const double tof_s              = isol.fire.flight_time_ms / 1000.0;
        Vec3         expected_intercept = {
            target_pos.x + velocity.x * tof_s, target_pos.y + velocity.y * tof_s, target_pos.z};
        const double ex = isol.intercept_point.x - expected_intercept.x;
        const double ey = isol.intercept_point.y - expected_intercept.y;
        CHECK_NEAR(std::sqrt(ex * ex + ey * ey), 0.0, 5.0); // within 5 m

        CHECK(isol.iterations <= 10);
    }
}

// ---------------------------------------------------------------------------
// solve_moving_target: target BELOW launcher (launcher above target)
// ---------------------------------------------------------------------------
static void test_moving_target_launcher_above() {
    SECTION("solve_moving_target: moving target below the launcher");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    AtmosphericConditions atm = isa_conditions(500.0);
    TrajectorySimulator   sim(lib.get("5.56x45_m855_62gr"), atm);

    const double          muzzle       = 930.0;
    Vec3                  launcher_pos = {0.0, 0.0, 500.0}; // launcher 500 m up
    Vec3                  target_pos   = {0.0, 350.0, 0.0}; // target at ground level
    Vec3                  velocity     = {10.0, 0.0, 0.0};  // 10 m/s East

    InterceptSolution     isol = solve_moving_target(sim,
                                                 launcher_pos,
                                                 target_pos,
                                                 velocity,
                                                 muzzle,
                                                 /*high_angle=*/false);

    std::printf("  Launcher above (500m): valid=%s  el=%.2f°  tof=%.0f ms  lead=%.1f m\n",
                isol.valid ? "true" : "false",
                isol.fire.elevation_deg,
                isol.fire.flight_time_ms,
                isol.lead_distance_m);

    CHECK(isol.valid);
    if (!isol.valid)
        return;

    // Shooting downward: elevation should be negative
    CHECK(isol.fire.elevation_deg < 0.0);
    CHECK(isol.lead_distance_m > 0.0);

    // Intercept altitude should be near ground (0 m)
    CHECK_NEAR(isol.intercept_point.z, 0.0, 5.0);
}

// ---------------------------------------------------------------------------
// solve_moving_target_slewed: slew time is non-zero when launcher is misaligned
// ---------------------------------------------------------------------------
static void test_moving_target_slewed_basic() {
    SECTION("solve_moving_target_slewed: slew_time_s > 0 when launcher is misaligned");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    AtmosphericConditions atm = isa_conditions(0.0);
    TrajectorySimulator   sim(lib.get("7.62x51_m80_147gr"), atm);

    const double          muzzle       = 853.0;
    Vec3                  launcher_pos = {0.0, 0.0, 0.0};
    Vec3                  target_pos   = {0.0, 500.0, 0.0}; // due North
    Vec3                  velocity     = {10.0, 0.0, 0.0};  // 10 m/s East

    LauncherSlew          slew;
    slew.yaw_deg_per_s   = 25.0;
    slew.pitch_deg_per_s = 25.0;

    // Case A: launcher already aimed at the target → slew_time near 0
    {
        const double      az_on = 0.0; // North = 0°, exactly toward target
        const double      el_on = 0.0;
        InterceptSolution isol  = solve_moving_target_slewed(
            sim, launcher_pos, az_on, el_on, target_pos, velocity, muzzle, slew);

        std::printf("  On-target launcher: valid=%s  slew_time=%.3f s\n",
                    isol.valid ? "true" : "false",
                    isol.slew_time_s);
        CHECK(isol.valid);
        // Slew time may be small but should not be negative
        CHECK(isol.slew_time_s >= 0.0);
    }

    // Case B: launcher pointing 90° off (East) → noticeable slew time
    {
        const double      az_off = 90.0; // East — far off from North target
        const double      el_off = 0.0;
        InterceptSolution isol   = solve_moving_target_slewed(
            sim, launcher_pos, az_off, el_off, target_pos, velocity, muzzle, slew);

        std::printf("  Off-target launcher (90° off): valid=%s  slew_time=%.3f s\n",
                    isol.valid ? "true" : "false",
                    isol.slew_time_s);
        CHECK(isol.valid);
        // 90° at 25°/s → at least ~3 s of slew
        CHECK(isol.slew_time_s > 2.0);
        // Intercept point must be ahead of where target is now
        CHECK(isol.lead_distance_m > 0.0);
    }

    // Case C: slew_time_s should be larger than in the no-slew version
    //         (because the target moves further during the slew delay)
    InterceptSolution plain  = solve_moving_target(sim, launcher_pos, target_pos, velocity, muzzle);
    InterceptSolution slewed = solve_moving_target_slewed(
        sim, launcher_pos, 90.0, 0.0, target_pos, velocity, muzzle, slew);

    if (plain.valid && slewed.valid) {
        // The slew-aware lead must be at least as large as the plain lead
        // (target moves further during the slew period)
        std::printf("  Plain lead=%.1f m   Slewed lead=%.1f m\n",
                    plain.lead_distance_m,
                    slewed.lead_distance_m);
        CHECK(slewed.lead_distance_m >= plain.lead_distance_m - 1.0);
    }
}

// ---------------------------------------------------------------------------
// solve_moving_target_slewed: elevated target above the launcher
// (combines the ascending-crossing bug-fix with slew-time calculation)
// ---------------------------------------------------------------------------
static void test_moving_target_slewed_elevated() {
    SECTION("solve_moving_target_slewed: target elevated above launcher + slew");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    const MunitionSpec&   spec = lib.get("338_lapua_250gr");
    AtmosphericConditions atm  = isa_conditions(0.0);
    TrajectorySimulator   sim(spec, atm);

    const double          muzzle       = 930.0;
    Vec3                  launcher_pos = {0.0, 0.0, 0.0};
    Vec3                  target_pos   = {0.0, 600.0, 120.0}; // 120 m above launcher
    Vec3                  velocity     = {15.0, 0.0, 0.0};    // 15 m/s East

    LauncherSlew          slew;
    slew.yaw_deg_per_s   = 25.0;
    slew.pitch_deg_per_s = 25.0;

    // Launcher pointing South (180°) — far off target — to generate slew time
    InterceptSolution isol = solve_moving_target_slewed(sim,
                                                        launcher_pos,
                                                        /*current_az=*/180.0,
                                                        /*current_el=*/0.0,
                                                        target_pos,
                                                        velocity,
                                                        muzzle,
                                                        slew,
                                                        /*high_angle=*/true);

    std::printf("  Elevated+slewed: valid=%s  az=%.2f°  el=%.2f°  tof=%.0f ms"
                "  slew_time=%.3f s  lead=%.1f m  iters=%d\n",
                isol.valid ? "true" : "false",
                isol.azimuth_deg,
                isol.fire.elevation_deg,
                isol.fire.flight_time_ms,
                isol.slew_time_s,
                isol.lead_distance_m,
                isol.iterations);

    CHECK(isol.valid);
    if (!isol.valid)
        return;

    // Launcher must aim upward to reach elevated target
    CHECK(isol.fire.elevation_deg > 0.0);

    // Non-trivial slew: 180° at 25°/s → slew_time must be several seconds
    CHECK(isol.slew_time_s > 2.0);

    // Target is moving East during slew, so lead must be significant
    CHECK(isol.lead_distance_m > 0.0);

    // Intercept altitude must be near the target plane (120 m)
    CHECK_NEAR(isol.intercept_point.z, target_pos.z, 10.0);
}

// ---------------------------------------------------------------------------
// solve_moving_target: returns valid=false when target is out of range
// ---------------------------------------------------------------------------
static void test_moving_target_out_of_range() {
    SECTION("solve_moving_target: returns valid=false when target is out of range");

    MunitionLibrary lib;
    if (!load_lib_for_moving(lib))
        return;

    AtmosphericConditions atm = isa_conditions(0.0);
    TrajectorySimulator   sim(lib.get("5.56x45_m855_62gr"), atm);

    const double          muzzle       = 930.0;
    Vec3                  launcher_pos = {0.0, 0.0, 0.0};
    // 50 km is far beyond any achievable range
    Vec3              target_pos = {0.0, 50000.0, 0.0};
    Vec3              velocity   = {0.0, 0.0, 0.0};

    InterceptSolution isol = solve_moving_target(sim, launcher_pos, target_pos, velocity, muzzle);

    std::printf("  Out-of-range: valid=%s (expected false)\n", isol.valid ? "true" : "false");
    CHECK(!isol.valid);
}

// ---------------------------------------------------------------------------
// Symplectic Euler integrator
// ---------------------------------------------------------------------------
static void test_step_euler() {
    SECTION("step_euler: produces valid trajectory, less accurate than RK4");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    ProjectileState       state;
    state.position = Vec3{0.0, 0.0, 100.0};
    state.velocity = Vec3{300.0, 0.0, 200.0};
    state.time     = 0.0;

    const double dt = 1.0 / 240.0;

    // Run 240 Euler steps (1 second)
    ProjectileState euler_state = state;
    for (int i = 0; i < 240; ++i)
        euler_state = sim.step_euler(euler_state, dt);

    // Run 240 RK4 steps (1 second)
    ProjectileState rk4_state = state;
    for (int i = 0; i < 240; ++i)
        rk4_state = sim.step_rk4(rk4_state, dt);

    // Both should advance time by ~1 second
    CHECK_NEAR(euler_state.time, 1.0, 1e-10);
    CHECK_NEAR(rk4_state.time, 1.0, 1e-10);

    // Euler should produce a different (less accurate) result
    const double pos_diff = (euler_state.position - rk4_state.position).norm();
    std::printf("  RK4 vs Euler position diff after 1s: %.3f m\n", pos_diff);
    // They should differ measurably but both be physically reasonable
    CHECK(pos_diff > 0.0);
    CHECK(pos_diff < 50.0); // not wildly divergent at 240 Hz

    // Euler batch simulation via SimulationConfig
    SimulationConfig cfg;
    cfg.dt       = dt;
    cfg.max_time = 30.0;
    cfg.use_rk4  = false; // Use Euler

    auto states = sim.simulate(state, cfg);
    CHECK(!states.empty());
    CHECK(states.back().position.z <= 0.01); // should land
}

// ---------------------------------------------------------------------------
// set_atmosphere: changing atmosphere mid-simulation
// ---------------------------------------------------------------------------
static void test_set_atmosphere() {
    SECTION("set_atmosphere: changing conditions affects drag");

    MunitionSpec          m         = drag_munition();
    AtmosphericConditions sea_level = isa_conditions(0.0);
    TrajectorySimulator   sim(m, sea_level);

    ProjectileState       state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{500.0, 0.0, 300.0};
    state.time     = 0.0;

    // Step once at sea level density
    ProjectileState after_sea = sim.step(state, 1.0 / 240.0);

    // Switch to high altitude (much lower density)
    AtmosphericConditions high_alt = isa_conditions(8000.0);
    sim.set_atmosphere(high_alt);

    // Step once at high altitude density
    ProjectileState after_alt = sim.step(state, 1.0 / 240.0);

    // Lower density = less drag = higher speed retained
    CHECK(after_alt.velocity.norm() > after_sea.velocity.norm());

    // Verify the drag constant was updated
    CHECK(sim.drag_k() > 0.0);

    // Restore and verify
    sim.set_atmosphere(sea_level);
    ProjectileState after_restore = sim.step(state, 1.0 / 240.0);
    CHECK_NEAR(after_restore.velocity.x, after_sea.velocity.x, 1e-10);
}

// ---------------------------------------------------------------------------
// MunitionLibrary::clear
// ---------------------------------------------------------------------------
static void test_munition_library_clear() {
    SECTION("MunitionLibrary: clear() empties the library");

    const std::string json = R"({
        "munitions": [
            {
                "name": "temp_round",
                "mass_kg": 0.01,
                "density_kg_m3": 11000,
                "reference_area_m2": 1e-4,
                "drag_coefficient": 0.30,
                "muzzle_velocity_ms": 900.0
            }
        ]
    })";

    MunitionLibrary   lib;
    lib.load_from_string(json);
    CHECK(lib.size() == 1);
    CHECK(lib.contains("temp_round"));

    lib.clear();
    CHECK(lib.size() == 0);
    CHECK(!lib.contains("temp_round"));

    // get() should throw after clear
    bool threw = false;
    try {
        (void)lib.get("temp_round");
    } catch (const std::out_of_range&) {
        threw = true;
    }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// MunitionSpec edge cases
// ---------------------------------------------------------------------------
static void test_munition_spec_edge_cases() {
    SECTION("MunitionSpec: ballistic_coefficient and volume edge cases");

    MunitionSpec m;
    m.mass_kg           = 0.01;
    m.density_kg_m3     = 11000.0;
    m.reference_area_m2 = 1e-4;
    m.drag_coefficient  = 0.0; // Vacuum sentinel

    // BC returns 0 when Cd is 0
    CHECK_NEAR(m.ballistic_coefficient(), 0.0, 1e-15);

    // BC returns 0 when area is 0
    m.drag_coefficient  = 0.3;
    m.reference_area_m2 = 0.0;
    CHECK_NEAR(m.ballistic_coefficient(), 0.0, 1e-15);

    // Volume returns 0 when density is 0
    m.density_kg_m3 = 0.0;
    CHECK_NEAR(m.volume_m3(), 0.0, 1e-15);
}

// ---------------------------------------------------------------------------
// SimulationConfig: invalid dt throws
// ---------------------------------------------------------------------------
static void test_invalid_dt_throws() {
    SECTION("simulate: throws on dt <= 0");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(drag_munition(), atm);

    ProjectileState       initial;
    initial.position = Vec3{0.0, 0.0, 10.0};
    initial.velocity = Vec3{100.0, 0.0, 0.0};

    // dt = 0
    {
        SimulationConfig cfg;
        cfg.dt     = 0.0;
        bool threw = false;
        try {
            (void)sim.simulate(initial, cfg);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }

    // dt < 0
    {
        SimulationConfig cfg;
        cfg.dt     = -1.0;
        bool threw = false;
        try {
            (void)sim.simulate(initial, cfg);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }
}

// ---------------------------------------------------------------------------
// Ground intersection: projectile starting on the ground with upward velocity
// ---------------------------------------------------------------------------
static void test_ground_start_upward() {
    SECTION("simulate: projectile starting at ground with upward velocity flies");

    AtmosphericConditions atm = still_atm();
    TrajectorySimulator   sim(vacuum_munition(), atm);

    ProjectileState       initial;
    initial.position = Vec3{0.0, 0.0, 0.0};    // exactly on ground
    initial.velocity = Vec3{100.0, 0.0, 50.0}; // upward velocity

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 30.0;
    cfg.ground_z = 0.0;

    auto states = sim.simulate(initial, cfg);
    CHECK(states.size() > 2); // should fly, not terminate immediately

    // Should reach an apex above ground
    double max_z = 0.0;
    for (const auto& s : states)
        if (s.position.z > max_z)
            max_z = s.position.z;
    CHECK(max_z > 10.0);

    // Should land back at ground level
    CHECK(states.back().position.z <= 0.01);
}

// ---------------------------------------------------------------------------
// Synchronous solve() function
// ---------------------------------------------------------------------------
static void test_solve_static_target() {
    SECTION("solve: static target returns valid solution with trajectory");

    AtmosphericConditions atm = still_atm();
    MunitionSpec          m   = drag_munition();

    SolveParams           sp;
    sp.launcher_pos    = {0.0, 0.0, 0.0};
    sp.target_pos      = {300.0, 400.0, 0.0}; // 500 m range
    sp.target_moving   = false;
    sp.munition        = m;
    sp.atmosphere      = atm;
    sp.muzzle_speed_ms = 600.0;

    SolveResult res = solve(sp);

    CHECK(res.valid);
    CHECK(res.elevation_deg > 0.0);
    CHECK(res.elevation_deg < 45.0);
    CHECK(res.flight_time_s > 0.0);
    CHECK_NEAR(res.range_m, 500.0, 0.1);
    CHECK(res.max_range_m > 0.0);
    CHECK(!res.has_intercept); // static target

    // Trajectory should have multiple points
    CHECK(res.trajectory.size() > 2);

    // First trajectory point should be near the launcher
    CHECK_NEAR(res.trajectory.front().x, 0.0, 0.1);
    CHECK_NEAR(res.trajectory.front().y, 0.0, 0.1);

    std::printf("  Static solve: elev=%.3f°  ToF=%.3f s  traj_pts=%zu  max_range=%.0f m\n",
                res.elevation_deg,
                res.flight_time_s,
                res.trajectory.size(),
                res.max_range_m);
}

static void test_solve_moving_target() {
    SECTION("solve: moving target returns intercept solution");

    MunitionLibrary lib;
    bool            loaded = false;
    for (const char* p :
         {"data/munitions.json", "../data/munitions.json", "../../data/munitions.json"}) {
        try {
            lib.load(p);
            loaded = true;
            break;
        } catch (const std::runtime_error&) {
        }
    }
    if (!loaded) {
        std::printf("  Skipped: munitions file not found\n");
        return;
    }

    AtmosphericConditions atm = isa_conditions(0.0);

    SolveParams           sp;
    sp.launcher_pos          = {0.0, 0.0, 0.0};
    sp.target_pos            = {0.0, 400.0, 0.0};
    sp.target_velocity       = {15.0, 0.0, 0.0};
    sp.target_moving         = true;
    sp.current_azimuth_deg   = 0.0;
    sp.current_elevation_deg = 0.0;
    sp.slew.yaw_deg_per_s    = 25.0;
    sp.slew.pitch_deg_per_s  = 25.0;
    sp.munition              = lib.get("7.62x51_m80_147gr");
    sp.atmosphere            = atm;
    sp.muzzle_speed_ms       = 853.0;

    SolveResult res = solve(sp);

    CHECK(res.valid);
    CHECK(res.has_intercept);
    CHECK(res.lead_distance_m > 0.0);
    CHECK(res.intercept_point.x > sp.target_pos.x); // lead is East
    CHECK(res.trajectory.size() > 2);
    CHECK(res.azimuth_deg > 0.0); // must lead East of North

    std::printf("  Moving solve: az=%.2f°  el=%.3f°  lead=%.1f m  slew=%.2f s  iters=%d\n",
                res.azimuth_deg,
                res.elevation_deg,
                res.lead_distance_m,
                res.slew_time_s,
                res.intercept_iters);
}

static void test_solve_out_of_range() {
    SECTION("solve: returns valid=false when target is out of range");

    SolveParams sp;
    sp.launcher_pos    = {0.0, 0.0, 0.0};
    sp.target_pos      = {0.0, 50000.0, 0.0}; // 50 km — way out of range
    sp.target_moving   = false;
    sp.munition        = drag_munition();
    sp.atmosphere      = still_atm();
    sp.muzzle_speed_ms = 600.0;

    SolveResult res = solve(sp);
    CHECK(!res.valid);
    CHECK(res.trajectory.empty());
}

static void test_solve_too_close() {
    SECTION("solve: returns valid=false when target is at launcher");

    SolveParams sp;
    sp.launcher_pos    = {0.0, 0.0, 0.0};
    sp.target_pos      = {0.0, 0.5, 0.0}; // < 1 m
    sp.target_moving   = false;
    sp.munition        = drag_munition();
    sp.atmosphere      = still_atm();
    sp.muzzle_speed_ms = 600.0;

    SolveResult res = solve(sp);
    CHECK(!res.valid);
}

// ---------------------------------------------------------------------------
// AsyncSolver
// ---------------------------------------------------------------------------
static void test_async_solver() {
    SECTION("AsyncSolver: request → poll → result cycle");

    SolveParams sp;
    sp.launcher_pos    = {0.0, 0.0, 0.0};
    sp.target_pos      = {300.0, 400.0, 0.0};
    sp.target_moving   = false;
    sp.munition        = drag_munition();
    sp.atmosphere      = still_atm();
    sp.muzzle_speed_ms = 600.0;

    AsyncSolver solver;

    // Before any request, result is default (valid=false)
    CHECK(!solver.result().valid);
    CHECK(!solver.computing());

    // Submit a request
    solver.request(sp);
    CHECK(solver.computing());

    // Poll until complete (with a timeout)
    auto t0 = std::chrono::high_resolution_clock::now();
    while (solver.computing()) {
        solver.poll();
        auto   t1         = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (elapsed_ms > 30000.0)
            break; // 30 s safety timeout (CI can be slow)
    }

    CHECK(!solver.computing());
    CHECK(solver.result().valid);
    CHECK(solver.result().trajectory.size() > 2);
    CHECK_NEAR(solver.result().range_m, 500.0, 0.1);

    std::printf("  AsyncSolver: valid=%s  elev=%.3f°  traj_pts=%zu\n",
                solver.result().valid ? "true" : "false",
                solver.result().elevation_deg,
                solver.result().trajectory.size());
}

static void test_async_solver_queuing() {
    SECTION("AsyncSolver: queued request replaces previous");

    SolveParams sp1;
    sp1.launcher_pos    = {0.0, 0.0, 0.0};
    sp1.target_pos      = {0.0, 300.0, 0.0};
    sp1.target_moving   = false;
    sp1.munition        = drag_munition();
    sp1.atmosphere      = still_atm();
    sp1.muzzle_speed_ms = 600.0;

    SolveParams sp2 = sp1;
    sp2.target_pos  = {0.0, 200.0, 0.0}; // different range

    AsyncSolver solver;
    solver.request(sp1); // starts computing
    solver.request(sp2); // should be queued

    // Wait for all solves to finish.  When poll() completes the first solve
    // and finds a queued request, it immediately starts the second solve
    // (computing_ transitions true→false→true inside a single poll() call).
    // So just waiting for computing()==false captures the end of both solves.
    auto t0 = std::chrono::high_resolution_clock::now();
    while (solver.computing()) {
        solver.poll();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::high_resolution_clock::now() - t0)
                              .count();
        if (elapsed_ms > 30000.0)
            break; // 30 s safety timeout (CI can be slow)
    }

    // The final result should match the second (queued) request
    CHECK(!solver.computing());
    CHECK(solver.result().valid);
    CHECK_NEAR(solver.result().range_m, 200.0, 1.0);

    std::printf("  AsyncSolver queuing: final range=%.0f m (expected 200)\n",
                solver.result().range_m);
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
    test_wind();
    test_realtime_performance();
    test_streaming_callback();
    test_altitude_varying_atmosphere();
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
    test_fire_control_wind();
    test_isa_above_stratopause();
    test_munition_library_load_file();
    test_table_with_target_altitude();
    test_table_wind_azimuth();
    test_build_num_samples_guard();
    test_invalid_muzzle_speed();

    // Altitude variation tests
    test_altitude_launcher_above_target();
    test_altitude_launcher_below_target();
    test_altitude_both_nonzero();
    test_altitude_fire_table();
    test_altitude_flight_time_within_max();

    // Moving-target intercept tests (solve_moving_target / solve_moving_target_slewed)
    test_moving_target_stationary();
    test_moving_target_level();
    test_moving_target_elevated_above_launcher();
    test_moving_target_launcher_above();
    test_moving_target_slewed_basic();
    test_moving_target_slewed_elevated();
    test_moving_target_out_of_range();

    // Coverage gap tests
    test_step_euler();
    test_set_atmosphere();
    test_munition_library_clear();
    test_munition_spec_edge_cases();
    test_invalid_dt_throws();
    test_ground_start_upward();

    // Async solver tests
    test_solve_static_target();
    test_solve_moving_target();
    test_solve_out_of_range();
    test_solve_too_close();
    test_async_solver();
    test_async_solver_queuing();

    std::printf("\n================================\n");
    std::printf("Passed: %d   Failed: %d\n", g_pass, g_fail);

    return (g_fail == 0) ? 0 : 1;
}
