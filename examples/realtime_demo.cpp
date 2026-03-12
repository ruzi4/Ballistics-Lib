/// @file realtime_demo.cpp
/// @brief Demonstrates real-time use of Ballistics-Lib at 60 Hz.
///
/// Simulates a 5.56 mm round fired at 45° elevation while measuring the
/// per-frame compute time to verify the 60 fps performance target.

#include <ballistics/ballistics.hpp>
#include <ballistics/math/math_constants.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ballistics;
using Clock = std::chrono::high_resolution_clock;

static double elapsed_us(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

int main(int argc, char* argv[]) {
    // -----------------------------------------------------------------------
    // 1. Load munition library from JSON
    // -----------------------------------------------------------------------
    const std::string data_path = (argc > 1)
        ? argv[1]
        : "data/munitions.json";

    MunitionLibrary lib;
    try {
        lib.load(data_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error loading munitions: %s\n", e.what());
        return 1;
    }

    std::printf("Loaded %zu munition(s):\n", lib.size());
    for (const auto& name : lib.names())
        std::printf("  %s\n", name.c_str());
    std::printf("\n");

    // -----------------------------------------------------------------------
    // 2. Build simulator
    // -----------------------------------------------------------------------
    const MunitionSpec& spec = lib.get("5.56x45_m855_62gr");

    std::printf("Munition : %s\n",   spec.name.c_str());
    std::printf("  mass   : %.4f kg\n", spec.mass_kg);
    std::printf("  density: %.0f kg/m³\n", spec.density_kg_m3);
    std::printf("  A_ref  : %.3e m²\n", spec.reference_area_m2);
    std::printf("  Cd     : %.3f\n", spec.drag_coefficient);
    std::printf("  BC     : %.2f kg/m²\n", spec.ballistic_coefficient());
    std::printf("\n");

    // ISA atmosphere at sea level with a 5 m/s headwind
    AtmosphericConditions atm = isa_conditions(/*altitude_m=*/0.0);
    atm.wind.velocity_ms = Vec3{-5.0, 0.0, 0.0}; // 5 m/s headwind on x-axis

    TrajectorySimulator sim(spec, atm);

    // -----------------------------------------------------------------------
    // 3. Initial state: 45° elevation, muzzle velocity 930 m/s
    // -----------------------------------------------------------------------
    const double muzzle_speed = 930.0; // m/s
    const double angle_rad    = 45.0 * kDegToRad;

    ProjectileState state;
    state.position = Vec3{0.0, 0.0, 1.5}; // 1.5 m above ground
    state.velocity = Vec3{muzzle_speed * std::cos(angle_rad), 0.0,
                          muzzle_speed * std::sin(angle_rad)};
    state.time = 0.0;

    // -----------------------------------------------------------------------
    // 4. Real-time loop at 60 Hz (simulated, no actual sleep)
    // -----------------------------------------------------------------------
    // Physics sub-step rate: 240 Hz (4 RK4 steps per render frame)
    const double render_dt = 1.0 / 60.0;
    const double phys_dt   = 1.0 / 240.0;
    const int    sub_steps = static_cast<int>(render_dt / phys_dt);

    const double budget_us = 1e6 / 60.0; // 16 667 µs per frame

    std::printf("%-6s  %-10s %-10s %-10s %-10s %-10s\n",
                "Frame", "Time(s)", "x(m)", "z(m)", "Speed(m/s)", "dt(µs)");
    std::printf("%s\n", std::string(66, '-').c_str());

    int  frame   = 0;
    bool landed  = false;

    while (!landed && state.time < 300.0) {
        auto t0 = Clock::now();

        // 4 physics sub-steps per render frame
        for (int i = 0; i < sub_steps && !landed; ++i) {
            const ProjectileState prev = state;
            state = sim.step(state, phys_dt);
            if (state.position.z <= 0.0) {
                // Linearly interpolate to the exact ground crossing so the
                // reported impact time and position are accurate rather than
                // being the first below-ground state.
                const double dz = prev.position.z - state.position.z;
                if (dz > 0.0) {
                    const double frac = prev.position.z / dz;
                    state.position = prev.position + frac * (state.position - prev.position);
                    state.velocity = prev.velocity + frac * (state.velocity - prev.velocity);
                    state.time     = prev.time     + frac * phys_dt;
                    state.position.z = 0.0;
                }
                landed = true;
            }
        }

        auto t1 = Clock::now();
        const double step_us = elapsed_us(t0, t1);

        // Print every 10th frame to keep output readable
        if (frame % 10 == 0) {
            std::printf("%-6d  %-10.3f %-10.1f %-10.1f %-10.1f %-10.2f\n",
                        frame,
                        state.time,
                        state.position.x,
                        state.position.z,
                        state.velocity.norm(),
                        step_us);

            if (step_us > budget_us) {
                std::printf("         *** WARNING: exceeded 60 fps budget (%.1f µs > %.1f µs)\n",
                            step_us, budget_us);
            }
        }
        ++frame;
    }

    std::printf("\n");
    std::printf("Impact:\n");
    std::printf("  time     : %.3f s\n",   state.time);
    std::printf("  x range  : %.1f m\n",   state.position.x);
    std::printf("  speed    : %.1f m/s\n", state.velocity.norm());
    std::printf("  frames   : %d\n",       frame);

    // -----------------------------------------------------------------------
    // 5. Batch offline simulation for comparison
    // -----------------------------------------------------------------------
    std::printf("\n--- Offline batch simulation ---\n");

    ProjectileState initial;
    initial.position = Vec3{0.0, 0.0, 1.5};
    initial.velocity = Vec3{muzzle_speed * std::cos(angle_rad), 0.0,
                            muzzle_speed * std::sin(angle_rad)};
    initial.time = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 300.0;
    cfg.use_rk4  = true;

    auto t0 = Clock::now();
    auto states = sim.simulate(initial, cfg);
    auto t1 = Clock::now();

    std::printf("Computed %zu states in %.2f µs\n",
                states.size(), elapsed_us(t0, t1));
    std::printf("Impact at t=%.3f s, x=%.1f m\n",
                states.back().time, states.back().position.x);

    return 0;
}
