#include "ballistics/fire_control.hpp"

#include "ballistics/math/math_constants.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ballistics {

namespace {

// ---------------------------------------------------------------------------
// Internal: simulate one shot, return (horizontal_range_m, flight_time_s).
//
// launch_height_m is the height of the launcher ABOVE the target plane.
// Negative values mean the launcher is BELOW the target.
// target_z_m is the absolute altitude of the target plane.
// The projectile starts at absolute altitude (target_z_m + launch_height_m).
//
// Two detection modes depending on launcher/target geometry:
//
//   launcher >= target  (launch_height_m >= 0):
//     Standard descending detection — simulation stops when the projectile
//     descends through target_z_m.  This is the normal artillery/sniper case.
//
//   launcher < target   (launch_height_m < 0):
//     Ascending detection — the projectile starts below target_z_m and must
//     cross it going upward.  The simulation callback records the first
//     ascending crossing and stops early.  This is the only physically
//     reachable solution at typical engagement ranges when the target is
//     above the launcher (the descending crossing occurs at ranges far
//     exceeding any practical engagement distance).
// ---------------------------------------------------------------------------
struct ShotResult {
    double range_m;
    double time_s;
};

ShotResult shoot(const TrajectorySimulator& sim,
                 double                     azimuth_deg,
                 double                     elevation_deg,
                 double                     muzzle_speed_ms,
                 double                     launch_height_m,
                 double                     target_z_m          = 0.0,
                 double                     sim_dt              = 1.0 / 240.0,
                 bool                       ascending_detection = false) {
    const double    azimuth_rad      = azimuth_deg * kDegToRad;
    const double    elevation_rad    = elevation_deg * kDegToRad;
    const double    launch_altitude_m = target_z_m + launch_height_m;

    const Vec3      horizontal_direction{std::sin(azimuth_rad), std::cos(azimuth_rad), 0.0};

    ProjectileState initial_state;
    initial_state.position = Vec3{0.0, 0.0, launch_altitude_m};
    initial_state.velocity = muzzle_speed_ms * (std::cos(elevation_rad) * horizontal_direction
                                                + Vec3{0.0, 0.0, std::sin(elevation_rad)});
    initial_state.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = sim_dt;
    cfg.max_time = 60.0; // 1-minute ceiling; enough for any munition at max elevation
    cfg.use_rk4  = true;

    // -----------------------------------------------------------------------
    // Launcher at or above target: standard descending detection
    // -----------------------------------------------------------------------
    if (launch_altitude_m >= target_z_m) {
        cfg.ground_z = target_z_m;

        ProjectileState final_state = initial_state;
        sim.simulate(initial_state, cfg, [&final_state](const ProjectileState& s) {
            final_state = s;
            return true;
        });

        // A valid impact clamps final_state.position.z exactly to target_z_m.
        // If it differs by more than 1 m the projectile never crossed the
        // target plane (simulation timed out or elevation was too shallow).
        if (std::abs(final_state.position.z - target_z_m) > 1.0)
            return {0.0, 0.0};

        const double horizontal_range_m =
            std::sqrt(final_state.position.x * final_state.position.x +
                      final_state.position.y * final_state.position.y);
        return {horizontal_range_m, final_state.time};
    }

    // -----------------------------------------------------------------------
    // Launcher below target: two detection modes
    // -----------------------------------------------------------------------
    if (ascending_detection) {
        // Ascending detection: record the first upward crossing of target_z_m.
        // Used for low-angle (direct) shots where the projectile clips target
        // altitude on the way up.  Set the floor below the launcher so
        // simulate() does not terminate early via the ground_z check.
        cfg.ground_z = launch_altitude_m - 1.0;

        ProjectileState prev_state   = initial_state;
        ProjectileState impact_state = initial_state;
        bool            found        = false;

        sim.simulate(initial_state, cfg, [&](const ProjectileState& s) {
            // Ascending crossing: prev_state was below target altitude, s is at/above it.
            if (prev_state.position.z < target_z_m && s.position.z >= target_z_m) {
                const double altitude_delta_m   = s.position.z - prev_state.position.z;
                const double crossing_fraction  = (altitude_delta_m > 0.0)
                                                      ? (target_z_m - prev_state.position.z) / altitude_delta_m
                                                      : 0.0;
                impact_state.position.x = prev_state.position.x + crossing_fraction * (s.position.x - prev_state.position.x);
                impact_state.position.y = prev_state.position.y + crossing_fraction * (s.position.y - prev_state.position.y);
                impact_state.position.z = target_z_m;
                impact_state.time       = prev_state.time + crossing_fraction * (s.time - prev_state.time);
                found                   = true;
                return false; // stop simulation early
            }
            // Abort as soon as the projectile starts falling back below target
            // altitude — it will never cross target_z_m going up again.
            if (s.position.z < target_z_m && s.velocity.z < 0.0)
                return false;
            prev_state = s;
            return true;
        });

        if (!found)
            return {0.0, 0.0};

        const double horizontal_range_m =
            std::sqrt(impact_state.position.x * impact_state.position.x +
                      impact_state.position.y * impact_state.position.y);
        return {horizontal_range_m, impact_state.time};
    }

    // Descending detection: the projectile arcs above target_z_m and is
    // detected when it crosses back down through target_z_m (plunging fire).
    // Setting ground_z = target_z_m lets simulate()'s built-in ground
    // intersection code detect the descending crossing automatically.
    // The projectile starts below target_z_m so the ascending pass is ignored
    // by the intersection check (which only fires when state.z > ground_z).
    cfg.ground_z = target_z_m;

    ProjectileState final_state = initial_state;
    sim.simulate(initial_state, cfg, [&final_state](const ProjectileState& s) {
        final_state = s;
        return true;
    });

    if (std::abs(final_state.position.z - target_z_m) > 1.0)
        return {0.0, 0.0};

    const double horizontal_range_m =
        std::sqrt(final_state.position.x * final_state.position.x +
                  final_state.position.y * final_state.position.y);
    return {horizontal_range_m, final_state.time};
}

// ---------------------------------------------------------------------------
// Internal: ternary search for the elevation angle of maximum range.
// Returns {theta_max_deg, max_range_m}.
// ---------------------------------------------------------------------------
std::pair<double, double> find_max_range_angle(const TrajectorySimulator& sim,
                                               double                     azimuth_deg,
                                               double                     muzzle_speed_ms,
                                               double                     launch_height_m,
                                               double                     search_lo_deg,
                                               double                     search_hi_deg,
                                               double                     target_z_m         = 0.0,
                                               int                        iterations         = 60,
                                               bool                       ascending_detection = false) {
    for (int i = 0; i < iterations; ++i) {
        const double search_span_deg      = search_hi_deg - search_lo_deg;
        const double elevation_lower_deg  = search_lo_deg + search_span_deg / 3.0;
        const double elevation_upper_deg  = search_hi_deg - search_span_deg / 3.0;
        const double range_at_lower_m     = shoot(sim, azimuth_deg, elevation_lower_deg, muzzle_speed_ms,
                                                  launch_height_m, target_z_m, 1.0 / 240.0,
                                                  ascending_detection).range_m;
        const double range_at_upper_m     = shoot(sim, azimuth_deg, elevation_upper_deg, muzzle_speed_ms,
                                                  launch_height_m, target_z_m, 1.0 / 240.0,
                                                  ascending_detection).range_m;
        if (range_at_lower_m < range_at_upper_m)
            search_lo_deg = elevation_lower_deg;
        else
            search_hi_deg = elevation_upper_deg;
    }
    const double max_range_elevation_deg = (search_lo_deg + search_hi_deg) * 0.5;
    const double max_range_m             = shoot(sim, azimuth_deg, max_range_elevation_deg,
                                                 muzzle_speed_ms, launch_height_m, target_z_m,
                                                 1.0 / 240.0, ascending_detection).range_m;
    return {max_range_elevation_deg, max_range_m};
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// solve_elevation
// ---------------------------------------------------------------------------

FireSolution solve_elevation(const TrajectorySimulator& sim,
                             LauncherOrientation        orientation,
                             double                     range_m,
                             double                     muzzle_speed_ms,
                             double                     launch_height_m,
                             bool                       high_angle,
                             double                     tolerance_m,
                             double                     target_altitude_m) {
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");

    // Search bounds: ±87° (3° inside vertical on both sides).
    //
    // The lower bound was previously −45°, which is insufficient when the
    // launcher sits well above the target: the maximum-range angle can then
    // be more negative than −45°, causing the ternary search to saturate at
    // the boundary and the bisection interval to collapse to a single point.
    // Extending to −87° makes the bound symmetric with the upper limit and
    // accommodates any realistic launcher-above-target geometry.
    constexpr double kLoSearch = -87.0;
    constexpr double kHiSearch = 87.0;

    const double azimuth_deg_local = orientation.azimuth_deg;

    // Bisection helper: given a fixed detection mode and search interval,
    // returns the best FireSolution found (invalid if error > 50×tolerance).
    // effective_hi_ang=false → low-angle search [lo_bound, max_range_elevation].
    // effective_hi_ang=true  → high-angle search [max_range_elevation, hi_bound].
    auto bisect = [&](bool asc_detect, bool eff_hi_ang) -> FireSolution {
        auto [max_range_elevation_deg, max_achievable_range_m] =
            find_max_range_angle(sim,
                                 azimuth_deg_local,
                                 muzzle_speed_ms,
                                 launch_height_m,
                                 kLoSearch,
                                 kHiSearch,
                                 target_altitude_m,
                                 60,
                                 asc_detect);
        if (range_m > max_achievable_range_m)
            return {};

        double bisect_lo_deg = eff_hi_ang ? max_range_elevation_deg : kLoSearch;
        double bisect_hi_deg = eff_hi_ang ? kHiSearch : max_range_elevation_deg;

        double best_elevation_deg  = (bisect_lo_deg + bisect_hi_deg) * 0.5;
        double best_flight_time_s  = 0.0;
        double best_range_error_m  = std::numeric_limits<double>::max();

        for (int i = 0; i < 60; ++i) {
            const double mid_elevation_deg = (bisect_lo_deg + bisect_hi_deg) * 0.5;
            const auto   shot_result       = shoot(sim,
                                                   azimuth_deg_local,
                                                   mid_elevation_deg,
                                                   muzzle_speed_ms,
                                                   launch_height_m,
                                                   target_altitude_m,
                                                   1.0 / 240.0,
                                                   asc_detect);

            const double range_error_m = std::abs(shot_result.range_m - range_m);
            if (range_error_m < best_range_error_m) {
                best_range_error_m = range_error_m;
                best_elevation_deg = mid_elevation_deg;
                best_flight_time_s = shot_result.time_s;
            }
            if (range_error_m <= tolerance_m)
                break;

            if (!eff_hi_ang) {
                if (shot_result.range_m < range_m) bisect_lo_deg = mid_elevation_deg;
                else                               bisect_hi_deg = mid_elevation_deg;
            } else {
                if (shot_result.range_m > range_m) bisect_lo_deg = mid_elevation_deg;
                else                               bisect_hi_deg = mid_elevation_deg;
            }
        }

        if (best_range_error_m > tolerance_m * 50.0)
            return {};
        return FireSolution{best_elevation_deg, best_flight_time_s * 1000.0, true};
    };

    // high_angle=true: plunging fire — descending detection, above-theta_max
    // search.  Used for intentional high-angle indirect fire.
    if (high_angle)
        return bisect(/*asc_detect=*/false, /*eff_hi_ang=*/true);

    // high_angle=false: prefer the natural low-angle direct-fire trajectory
    // (descending detection, below-theta_max search).  This handles both the
    // launcher-above-target case and the launcher-slightly-below-target case
    // where the bullet arcs above target altitude and descends to it.
    //
    // If descending detection cannot reach the requested range (e.g. the
    // target is far above the launcher and the minimum descending-crossing
    // range is larger than range_m), fall back to ascending detection.  That
    // mode finds the short-range solution where the bullet clips target
    // altitude on the way up.
    if (FireSolution sol = bisect(/*asc_detect=*/false, /*eff_hi_ang=*/false); sol.valid)
        return sol;

    // Ascending-detection fallback (launcher below target, short range).
    if (launch_height_m < 0.0)
        return bisect(/*asc_detect=*/true, /*eff_hi_ang=*/true);

    return {};
}

// ---------------------------------------------------------------------------
// solve_moving_target
// ---------------------------------------------------------------------------
// Algorithm: iterative fixed-point.
//
//   1. Solve elevation to the *current* target position → flight time T₀.
//   2. Project the target forward: intercept = target_pos + target_vel × Tₙ.
//   3. Solve elevation to the *projected* intercept → new flight time Tₙ₊₁.
//   4. Repeat from step 2 until |Tₙ₊₁ − Tₙ| < 1 ms or max_iterations.
//
// The azimuth must be recomputed at each iteration because the intercept
// point's bearing from the launcher changes as the lead grows.
// ---------------------------------------------------------------------------

InterceptSolution solve_moving_target(const TrajectorySimulator& sim,
                                      const Vec3&                launcher_pos,
                                      const Vec3&                target_pos,
                                      const Vec3&                target_velocity,
                                      double                     muzzle_speed_ms,
                                      bool                       high_angle,
                                      double                     tolerance_m,
                                      int                        max_iterations) {
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");

    // --- Step 1: initial solve to the current target position ---------------
    const double delta_east_m  = target_pos.x - launcher_pos.x;
    const double delta_north_m = target_pos.y - launcher_pos.y;
    const double initial_range_m = std::sqrt(delta_east_m * delta_east_m + delta_north_m * delta_north_m);

    if (initial_range_m < 1.0)
        return {}; // launcher and target essentially co-located

    const double        initial_azimuth_deg    = std::atan2(delta_east_m, delta_north_m) * kRadToDeg;
    const double        initial_launch_height_m = launcher_pos.z - target_pos.z;

    LauncherOrientation initial_orient;
    initial_orient.azimuth_deg = initial_azimuth_deg;

    FireSolution initial_fire_sol = solve_elevation(
        sim, initial_orient, initial_range_m, muzzle_speed_ms, initial_launch_height_m,
        high_angle, tolerance_m, target_pos.z);
    if (!initial_fire_sol.valid)
        return {}; // target already out of range

    // Convergence loop
    double       flight_time_s    = initial_fire_sol.flight_time_ms / 1000.0;

    FireSolution best_fire_sol    = initial_fire_sol;
    double       best_azimuth_deg = initial_azimuth_deg;
    Vec3         best_intercept   = target_pos;
    int          best_iter_count  = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // --- Step 2: project target position --------------------------------
        const Vec3 intercept_pos = {target_pos.x + target_velocity.x * flight_time_s,
                                    target_pos.y + target_velocity.y * flight_time_s,
                                    target_pos.z + target_velocity.z * flight_time_s};

        // --- Step 3: solve to the projected intercept point -----------------
        const double intercept_delta_east_m  = intercept_pos.x - launcher_pos.x;
        const double intercept_delta_north_m = intercept_pos.y - launcher_pos.y;
        const double intercept_range_m       = std::sqrt(intercept_delta_east_m * intercept_delta_east_m +
                                                          intercept_delta_north_m * intercept_delta_north_m);

        if (intercept_range_m < 1.0)
            return {}; // intercept collapsed onto launcher

        const double        intercept_azimuth_deg    = std::atan2(intercept_delta_east_m, intercept_delta_north_m) * kRadToDeg;
        const double        intercept_launch_height_m = launcher_pos.z - intercept_pos.z;

        LauncherOrientation intercept_orient;
        intercept_orient.azimuth_deg = intercept_azimuth_deg;

        FireSolution intercept_fire_sol =
            solve_elevation(sim, intercept_orient, intercept_range_m, muzzle_speed_ms,
                            intercept_launch_height_m, high_angle, tolerance_m, intercept_pos.z);
        if (!intercept_fire_sol.valid)
            return {}; // intercept point is out of range

        const double new_flight_time_s = intercept_fire_sol.flight_time_ms / 1000.0;

        best_fire_sol    = intercept_fire_sol;
        best_azimuth_deg = intercept_azimuth_deg;
        best_intercept   = intercept_pos;
        best_iter_count  = iter + 1;

        // --- Step 4: check convergence (1 ms threshold) ---------------------
        if (std::abs(new_flight_time_s - flight_time_s) < 0.001)
            break;

        flight_time_s = new_flight_time_s;
    }

    // Compute horizontal lead distance
    const double      lead_delta_east_m  = best_intercept.x - target_pos.x;
    const double      lead_delta_north_m = best_intercept.y - target_pos.y;
    const double      lead_distance_m    = std::sqrt(lead_delta_east_m * lead_delta_east_m +
                                                      lead_delta_north_m * lead_delta_north_m);

    InterceptSolution result;
    result.fire            = best_fire_sol;
    result.azimuth_deg     = best_azimuth_deg;
    result.intercept_point = best_intercept;
    result.lead_distance_m = lead_distance_m;
    result.iterations      = best_iter_count;
    result.valid           = true;
    return result;
}

// ---------------------------------------------------------------------------
// solve_moving_target_slewed
// ---------------------------------------------------------------------------
// Outer fixed-point wrapping solve_moving_target().
//
// A physical launcher cannot snap instantly to a new firing angle; it must
// slew (rotate) at its rated speed.  During that slew the target keeps
// moving, so the effective fire point is target_pos + target_vel * T_slew.
// The required T_slew itself depends on the firing angle, which depends on
// the intercept computed from the fire point — a fixed-point problem.
//
// Convergence is fast (≤ 5 iterations) for typical tactical scenarios.
// ---------------------------------------------------------------------------

InterceptSolution solve_moving_target_slewed(const TrajectorySimulator& sim,
                                             const Vec3&                launcher_pos,
                                             double                     current_azimuth_deg,
                                             double                     current_elevation_deg,
                                             const Vec3&                target_pos,
                                             const Vec3&                target_velocity,
                                             double                     muzzle_speed_ms,
                                             const LauncherSlew&        slew,
                                             bool                       high_angle,
                                             double                     tolerance_m,
                                             int                        max_iterations) {
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");

    // Compute the shortest angular distance between two azimuths [0, 180].
    auto az_delta = [](double a, double b) -> double {
        double d = std::abs(a - b);
        return (d > 180.0) ? 360.0 - d : d;
    };

    // Time for the launcher to slew from current orientation to (target_az, target_el).
    auto slew_time_for = [&](double target_az, double target_el) -> double {
        const double t_yaw = (slew.yaw_deg_per_s > 0.0)
                                 ? az_delta(current_azimuth_deg, target_az) / slew.yaw_deg_per_s
                                 : 0.0;
        const double t_pitch =
            (slew.pitch_deg_per_s > 0.0)
                ? std::abs(current_elevation_deg - target_el) / slew.pitch_deg_per_s
                : 0.0;
        return std::max(t_yaw, t_pitch);
    };

    double            slew_time_estimate_s = 0.0; // current slew-time estimate (seconds)
    InterceptSolution best;

    for (int iter = 0; iter <= max_iterations; ++iter) {
        // Project target to where it will be when the launcher finishes slewing.
        const Vec3 deferred_target_pos = {target_pos.x + target_velocity.x * slew_time_estimate_s,
                                          target_pos.y + target_velocity.y * slew_time_estimate_s,
                                          target_pos.z + target_velocity.z * slew_time_estimate_s};

        // Solve intercept from that deferred fire position.
        InterceptSolution intercept_sol = solve_moving_target(sim,
                                                              launcher_pos,
                                                              deferred_target_pos,
                                                              target_velocity,
                                                              muzzle_speed_ms,
                                                              high_angle,
                                                              tolerance_m,
                                                              max_iterations);

        if (!intercept_sol.valid)
            return {};

        // Update slew estimate from the newly computed firing angles.
        const double updated_slew_time_s  = slew_time_for(intercept_sol.azimuth_deg,
                                                            intercept_sol.fire.elevation_deg);
        intercept_sol.slew_time_s         = updated_slew_time_s;
        best                              = intercept_sol;

        if (std::abs(updated_slew_time_s - slew_time_estimate_s) < 0.01)
            break; // converged (10 ms threshold)
        slew_time_estimate_s = updated_slew_time_s;
    }

    return best;
}

// ---------------------------------------------------------------------------
// FireControlTable::build
// ---------------------------------------------------------------------------
// Strategy: sweep num_samples elevation angles across the full search range,
// simulate each, then extract the monotone portion that corresponds to the
// requested solution type (low-angle or high-angle).
//
// Cost: num_samples full trajectory simulations — far cheaper than calling
// solve_elevation() (which uses ~180 simulations) for every range value.
// Typical wall time: 50–300 ms.  Intended for startup / async use.
//
// The internal dt is coarsened to 1/120 Hz for build speed; interpolation
// at lookup absorbs the small additional error.
// ---------------------------------------------------------------------------

void FireControlTable::build(const TrajectorySimulator& sim,
                             double                     muzzle_speed_ms,
                             double                     azimuth_deg,
                             double                     launch_height_m,
                             bool                       high_angle,
                             int                        num_samples,
                             double                     target_altitude_m) {
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");
    if (num_samples < 2)
        num_samples = 2;

    // Symmetric bounds (see solve_elevation for the rationale behind −87°).
    constexpr double kLoSearch = -87.0;
    constexpr double kHiSearch = 87.0;

    // Use a coarser dt during the sweep — accurate enough for table use
    constexpr double kBuildDt = 1.0 / 120.0;

    // Detection mode mirrors solve_elevation:
    //   high_angle=true  → descending detection, sweep above theta_max (plunging fire)
    //   high_angle=false → descending detection, sweep below theta_max (direct fire)
    // Ascending detection is not used in the table; solve_elevation handles the
    // short-range ascending-path fallback internally.
    const bool ascending_detection = false;
    const bool effective_hi_ang    = high_angle;

    // --- Step 1: quick ternary search for theta_max (20 iterations = 40 sims) ---
    auto [theta_max_sb, r_max_unused] = find_max_range_angle(sim,
                                                             azimuth_deg,
                                                             muzzle_speed_ms,
                                                             launch_height_m,
                                                             kLoSearch,
                                                             kHiSearch,
                                                             target_altitude_m,
                                                             /*iterations=*/20,
                                                             ascending_detection);
    (void)r_max_unused;
    // Copy out of the structured binding into a plain variable.
    // C++17 does not allow lambdas to capture structured bindings directly;
    // that was only standardised in C++20.
    const double theta_max = theta_max_sb;

    // Determine the effective lower bound of the sweep by locating the first
    // angle that yields a positive range.
    const double sweep_elevation_lo_deg = [&]() -> double {
        if (effective_hi_ang)
            return theta_max;
        double bisect_lo_deg = kLoSearch, bisect_hi_deg = theta_max;
        for (int i = 0; i < 20; ++i) {
            const double bisect_mid_deg = (bisect_lo_deg + bisect_hi_deg) * 0.5;
            const double shot_range_m   = shoot(sim,
                                                azimuth_deg,
                                                bisect_mid_deg,
                                                muzzle_speed_ms,
                                                launch_height_m,
                                                target_altitude_m,
                                                kBuildDt,
                                                ascending_detection).range_m;
            if (shot_range_m > 0.0)
                bisect_hi_deg = bisect_mid_deg;
            else
                bisect_lo_deg = bisect_mid_deg;
        }
        return bisect_lo_deg;
    }();
    const double sweep_elevation_hi_deg = effective_hi_ang ? kHiSearch : theta_max;
    const double elevation_step_deg     = (sweep_elevation_hi_deg - sweep_elevation_lo_deg) / (num_samples - 1);

    // --- Step 2: sweep the selected arc ---
    struct Raw {
        double elev;
        double range;
        double tof;
    };
    std::vector<Raw> raw;
    raw.reserve(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        const double elevation_deg = sweep_elevation_lo_deg + i * elevation_step_deg;
        const auto   shot_result   = shoot(sim,
                                           azimuth_deg,
                                           elevation_deg,
                                           muzzle_speed_ms,
                                           launch_height_m,
                                           target_altitude_m,
                                           kBuildDt,
                                           ascending_detection);
        raw.push_back({elevation_deg, shot_result.range_m, shot_result.time_s});
    }

    // --- Step 3: build entries in ascending range order ---
    // Low-angle         : range increases with elevation → forward pass
    // High-angle or
    // ascending mode    : range decreases with elevation → reverse pass
    entries_.clear();
    entries_.reserve(num_samples);

    if (!effective_hi_ang) {
        double prev_range_m = -1.0;
        for (int i = 0; i < num_samples; ++i) {
            if (raw[i].range > prev_range_m) {
                entries_.push_back({raw[i].range, raw[i].elev, raw[i].tof * 1000.0});
                prev_range_m = raw[i].range;
            }
        }
    } else {
        double prev_range_m = -1.0;
        for (int i = num_samples - 1; i >= 0; --i) {
            if (raw[i].range > prev_range_m) {
                entries_.push_back({raw[i].range, raw[i].elev, raw[i].tof * 1000.0});
                prev_range_m = raw[i].range;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// FireControlTable::lookup
// ---------------------------------------------------------------------------

FireSolution FireControlTable::lookup(double range_m) const {
    if (entries_.empty() || range_m > entries_.back().range_m || range_m < 0.0)
        return {};

    // Binary search for the first entry with range >= range_m
    auto it =
        std::lower_bound(entries_.begin(), entries_.end(), range_m, [](const Entry& e, double r) {
            return e.range_m < r;
        });

    // Below the minimum range stored in the table → no valid solution
    if (it == entries_.begin() && it->range_m != range_m)
        return {};

    // Exact match
    if (it->range_m == range_m)
        return FireSolution{it->elevation_deg, it->flight_time_ms, true};

    // Interpolate between [lower_entry, upper_entry]
    const Entry& upper_entry = *it;
    const Entry& lower_entry = *std::prev(it);

    const double range_span_m = upper_entry.range_m - lower_entry.range_m;
    // Guard against duplicate range entries (can occur due to floating-point
    // rounding in the sweep).  Return the lower entry directly.
    if (range_span_m == 0.0)
        return FireSolution{lower_entry.elevation_deg, lower_entry.flight_time_ms, true};

    const double interp_fraction = (range_m - lower_entry.range_m) / range_span_m;

    return FireSolution{lower_entry.elevation_deg + interp_fraction * (upper_entry.elevation_deg - lower_entry.elevation_deg),
                        lower_entry.flight_time_ms + interp_fraction * (upper_entry.flight_time_ms - lower_entry.flight_time_ms),
                        true};
}

// ---------------------------------------------------------------------------
// FireControlTable::max_range_m
// ---------------------------------------------------------------------------

double FireControlTable::max_range_m() const noexcept {
    return entries_.empty() ? 0.0 : entries_.back().range_m;
}

} // namespace ballistics
