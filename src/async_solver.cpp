#include "ballistics/async_solver.hpp"

#include "ballistics/math/math_constants.hpp"

#include <algorithm>
#include <cmath>

namespace ballistics {

// ---------------------------------------------------------------------------
// solve — synchronous fire-control computation
// ---------------------------------------------------------------------------

SolveResult solve(const SolveParams& p) {
    SolveResult  out;

    const double dx      = p.target_pos.x - p.launcher_pos.x;
    const double dy      = p.target_pos.y - p.launcher_pos.y;
    const double range_m = std::sqrt(dx * dx + dy * dy);
    out.range_m          = range_m;

    out.alt_diff_m = p.launcher_pos.z - p.target_pos.z;

    if (range_m < 1.0)
        return out; // launcher and target too close

    TrajectorySimulator sim(p.munition, p.atmosphere);

    // Build a quick table to find max range for diagnostics
    const double     az_diag       = std::atan2(dx, dy) * kRadToDeg;
    const double     launch_height = p.launcher_pos.z - p.target_pos.z;
    const double     target_alt    = p.target_pos.z;

    FireControlTable diag_table;
    diag_table.build(sim, p.muzzle_speed_ms, az_diag, launch_height, false, 50, target_alt);
    out.max_range_m = diag_table.max_range_m();

    // -------------------------------------------------------------------
    // Choose solver: moving-target intercept or static elevation solve
    // -------------------------------------------------------------------
    double used_az  = az_diag;
    double used_el  = 0.0;
    double used_tof = 0.0;
    Vec3   aim_pos  = p.target_pos;

    if (p.target_moving) {
        InterceptSolution isol = solve_moving_target_slewed(sim,
                                                            p.launcher_pos,
                                                            p.current_azimuth_deg,
                                                            p.current_elevation_deg,
                                                            p.target_pos,
                                                            p.target_velocity,
                                                            p.muzzle_speed_ms,
                                                            p.slew,
                                                            /*high_angle=*/false,
                                                            /*tolerance_m=*/0.5,
                                                            /*max_iterations=*/10);

        if (!isol.valid)
            return out;

        used_az  = isol.azimuth_deg;
        used_el  = isol.fire.elevation_deg;
        used_tof = isol.fire.flight_time_ms / 1000.0;
        aim_pos  = isol.intercept_point;

        out.has_intercept   = true;
        out.intercept_point = isol.intercept_point;
        out.lead_distance_m = isol.lead_distance_m;
        out.slew_time_s     = isol.slew_time_s;
        out.intercept_iters = isol.iterations;
    } else {
        LauncherOrientation orient;
        orient.azimuth_deg = az_diag;

        FireSolution sol = solve_elevation(sim,
                                           orient,
                                           range_m,
                                           p.muzzle_speed_ms,
                                           launch_height,
                                           /*high_angle=*/false,
                                           /*tolerance_m=*/0.5,
                                           target_alt);

        // When the launcher is below the target, the low-angle ascending-path
        // solution may not reach the requested range.  Fall back to the
        // plunging-fire (high-angle) solution in that case.
        if (!sol.valid && launch_height < 0.0) {
            sol = solve_elevation(sim,
                                  orient,
                                  range_m,
                                  p.muzzle_speed_ms,
                                  launch_height,
                                  /*high_angle=*/true,
                                  /*tolerance_m=*/0.5,
                                  target_alt);
        }

        if (!sol.valid)
            return out;

        used_az  = az_diag;
        used_el  = sol.elevation_deg;
        used_tof = sol.flight_time_ms / 1000.0;
    }

    out.valid         = true;
    out.azimuth_deg   = used_az;
    out.elevation_deg = used_el;
    out.flight_time_s = used_tof;

    // Collect trajectory arc in ballistics coordinates
    const double    az_r = used_az * kDegToRad;
    const double    el_r = used_el * kDegToRad;
    const double    v    = p.muzzle_speed_ms;

    ProjectileState init;
    init.position = p.launcher_pos;
    init.velocity = {
        v * std::sin(az_r) * std::cos(el_r), // East  (+x)
        v * std::cos(az_r) * std::cos(el_r), // North (+y)
        v * std::sin(el_r)                   // Up    (+z)
    };
    init.time = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 60.0; // 60 Hz — fine for visualisation
    cfg.use_rk4  = true;
    cfg.max_time = used_tof * 1.5 + 2.0;
    // When the launcher is below the target the ascending solution reaches the
    // target plane on the way UP.  Setting ground_z to target altitude would
    // cause simulate() to terminate immediately (launcher starts below ground_z).
    // Use the lower of launcher and target altitude so the floor is always
    // below the launch point; the max_time cap stops the trajectory near impact.
    cfg.ground_z = std::min(p.launcher_pos.z, aim_pos.z) - 0.5;

    auto states = sim.simulate(init, cfg);
    out.trajectory.reserve(states.size());
    for (const auto& s : states)
        out.trajectory.push_back(s.position);

    return out;
}

// ---------------------------------------------------------------------------
// AsyncSolver
// ---------------------------------------------------------------------------

void AsyncSolver::request(const SolveParams& params) {
    if (computing_) {
        // Queue the request for when the current solve completes
        queued_        = true;
        queued_params_ = params;
        return;
    }

    computing_ = true;
    pending_   = std::async(std::launch::async, ballistics::solve, params);
}

bool AsyncSolver::poll() {
    if (!computing_) {
        // Start queued request if one is waiting
        if (queued_) {
            queued_    = false;
            computing_ = true;
            pending_   = std::async(std::launch::async, ballistics::solve, queued_params_);
        }
        return false;
    }

    if (!pending_.valid())
        return false;
    if (pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return false;

    // Solve finished — install new result
    current_   = pending_.get();
    computing_ = false;

    // Start queued request if one accumulated during this solve
    if (queued_) {
        queued_    = false;
        computing_ = true;
        pending_   = std::async(std::launch::async, ballistics::solve, queued_params_);
    }

    return true;
}

} // namespace ballistics
