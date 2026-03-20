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

    const double delta_east_m  = p.target_pos.x - p.launcher_pos.x;
    const double delta_north_m = p.target_pos.y - p.launcher_pos.y;
    const double range_m       = std::sqrt(delta_east_m * delta_east_m + delta_north_m * delta_north_m);
    out.range_m                = range_m;

    out.alt_diff_m = p.launcher_pos.z - p.target_pos.z;

    if (range_m < 1.0)
        return out; // launcher and target too close

    TrajectorySimulator sim(p.munition, p.atmosphere);

    // Build a quick table to find max range for diagnostics
    const double     azimuth_to_target_deg = std::atan2(delta_east_m, delta_north_m) * kRadToDeg;
    const double     launch_height_m       = p.launcher_pos.z - p.target_pos.z;
    const double     target_altitude_m     = p.target_pos.z;

    FireControlTable diag_table;
    diag_table.build(sim, p.muzzle_speed_ms, azimuth_to_target_deg, launch_height_m, false, 50, target_altitude_m);
    out.max_range_m = diag_table.max_range_m();

    // -------------------------------------------------------------------
    // Choose solver: moving-target intercept or static elevation solve
    // -------------------------------------------------------------------
    double fire_azimuth_deg   = azimuth_to_target_deg;
    double fire_elevation_deg = 0.0;
    double flight_time_s      = 0.0;
    Vec3   aim_position       = p.target_pos;

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

        fire_azimuth_deg   = isol.azimuth_deg;
        fire_elevation_deg = isol.fire.elevation_deg;
        flight_time_s      = isol.fire.flight_time_ms / 1000.0;
        aim_position       = isol.intercept_point;

        out.has_intercept   = true;
        out.intercept_point = isol.intercept_point;
        out.lead_distance_m = isol.lead_distance_m;
        out.slew_time_s     = isol.slew_time_s;
        out.intercept_iters = isol.iterations;
    } else {
        LauncherOrientation orient;
        orient.azimuth_deg = azimuth_to_target_deg;

        // solve_elevation with high_angle=false tries the natural low-angle
        // direct-fire path (descending detection) first.  If that cannot reach
        // the requested range it automatically falls back to the ascending-
        // detection path (short-range clip of target altitude on the way up).
        // No explicit fallback is needed here.
        //
        // Barrel-geometry correction: the trajectory starts at the muzzle, not
        // the launcher body centre.  The muzzle's Z and horizontal position
        // depend on the fire elevation, so we iterate:
        //   1. Solve elevation using current effective range / height.
        //   2. Compute the muzzle position from the solved elevation.
        //   3. Re-derive effective range and height from the muzzle to the target.
        //   4. Repeat until both values converge (typically 2–3 iterations).
        double       eff_range_m  = range_m;
        double       eff_height_m = launch_height_m;
        FireSolution sol;

        for (int barrel_iter = 0; barrel_iter < 5; ++barrel_iter) {
            sol = solve_elevation(sim,
                                  orient,
                                  eff_range_m,
                                  p.muzzle_speed_ms,
                                  eff_height_m,
                                  /*high_angle=*/false,
                                  /*tolerance_m=*/0.5,
                                  target_altitude_m);
            if (!sol.valid)
                return out;

            // Compute muzzle position at this elevation.
            const double elevation_rad = sol.elevation_deg * kDegToRad;
            const double azimuth_rad   = azimuth_to_target_deg * kDegToRad;
            const Vec3   firing_direction = {std::sin(azimuth_rad) * std::cos(elevation_rad),
                                             std::cos(azimuth_rad) * std::cos(elevation_rad),
                                             std::sin(elevation_rad)};
            const Vec3   muzzle_position  = p.launcher_pos
                                          + p.barrel_base_offset_m
                                          + firing_direction * p.barrel_length_m;

            // Effective range and height from muzzle to target.
            const double muzzle_delta_east_m  = p.target_pos.x - muzzle_position.x;
            const double muzzle_delta_north_m = p.target_pos.y - muzzle_position.y;
            const double range_from_muzzle_m  = std::sqrt(muzzle_delta_east_m * muzzle_delta_east_m +
                                                            muzzle_delta_north_m * muzzle_delta_north_m);
            const double height_from_muzzle_m = muzzle_position.z - p.target_pos.z;

            if (std::abs(range_from_muzzle_m - eff_range_m) < 0.01 &&
                std::abs(height_from_muzzle_m - eff_height_m) < 0.001)
                break;

            eff_range_m  = range_from_muzzle_m;
            eff_height_m = height_from_muzzle_m;
        }

        if (!sol.valid)
            return out;

        fire_azimuth_deg   = azimuth_to_target_deg;
        fire_elevation_deg = sol.elevation_deg;
        flight_time_s      = sol.flight_time_ms / 1000.0;
    }

    out.valid         = true;
    out.azimuth_deg   = fire_azimuth_deg;
    out.elevation_deg = fire_elevation_deg;
    out.flight_time_s = flight_time_s;

    // Collect trajectory arc in ballistics coordinates
    const double azimuth_rad   = fire_azimuth_deg * kDegToRad;
    const double elevation_rad = fire_elevation_deg * kDegToRad;

    // Unit vector in the fire-solution direction (ballistics: x=East, y=North, z=Up)
    const Vec3 firing_direction = {
        std::sin(azimuth_rad) * std::cos(elevation_rad), // East  (+x)
        std::cos(azimuth_rad) * std::cos(elevation_rad), // North (+y)
        std::sin(elevation_rad)                          // Up    (+z)
    };

    // Muzzle (barrel tip) position: launcher body centre
    //   + fixed offset to barrel pivot/base
    //   + barrel_length_m along the fire-solution direction
    const Vec3 muzzle_pos = p.launcher_pos
                          + p.barrel_base_offset_m
                          + firing_direction * p.barrel_length_m;
    out.muzzle_pos = muzzle_pos;

    ProjectileState initial_state;
    initial_state.position = muzzle_pos;
    initial_state.velocity = firing_direction * p.muzzle_speed_ms;
    initial_state.time     = 0.0;

    SimulationConfig traj_cfg;
    traj_cfg.dt       = 1.0 / 60.0; // 60 Hz — fine for visualisation
    traj_cfg.use_rk4  = true;
    traj_cfg.max_time = flight_time_s * 1.5 + 2.0;
    // When the launcher is below the target the ascending solution reaches the
    // target plane on the way UP.  Setting ground_z to target altitude would
    // cause simulate() to terminate immediately (launcher starts below ground_z).
    // Use the lower of launcher and target altitude so the floor is always
    // below the launch point; the max_time cap stops the trajectory near impact.
    traj_cfg.ground_z = std::min(p.launcher_pos.z, aim_position.z) - 0.5;

    auto states = sim.simulate(initial_state, traj_cfg);
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
