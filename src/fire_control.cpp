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
// Detection: always uses descending crossing of target_z_m.  When the
// launcher is below the target the projectile must arc above target_z_m
// before descending through it (plunging fire); shallow elevations that
// never reach target_z_m return {0, 0}.
// ---------------------------------------------------------------------------
struct ShotResult { double range_m; double time_s; };

ShotResult shoot(
    const TrajectorySimulator& sim,
    double azimuth_deg,
    double elevation_deg,
    double muzzle_speed_ms,
    double launch_height_m,
    double target_z_m  = 0.0,
    double sim_dt      = 1.0 / 240.0)
{
    const double az       = azimuth_deg   * kDegToRad;
    const double el       = elevation_deg * kDegToRad;
    const double launch_z = target_z_m + launch_height_m;

    const Vec3 h_dir{std::sin(az), std::cos(az), 0.0};

    ProjectileState st;
    st.position = Vec3{0.0, 0.0, launch_z};
    st.velocity = muzzle_speed_ms * (std::cos(el) * h_dir
                                   + Vec3{0.0, 0.0, std::sin(el)});
    st.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = sim_dt;
    cfg.max_time = 60.0;   // 1-minute ceiling; enough for any munition at max elevation
    cfg.use_rk4  = true;
    cfg.ground_z = target_z_m;

    ProjectileState last = st;
    sim.simulate(st, cfg, [&last](const ProjectileState& s) {
        last = s;
        return true;
    });

    // A valid impact clamps last.position.z exactly to target_z_m.
    // If it differs by more than 1 m the projectile never crossed the
    // target plane on the descending path (simulation timed out, elevation
    // was too shallow, or — for launcher-below-target — the projectile
    // never arced above target_z_m).
    if (std::abs(last.position.z - target_z_m) > 1.0)
        return {0.0, 0.0};

    const double h_range = std::sqrt(last.position.x * last.position.x
                                   + last.position.y * last.position.y);
    return {h_range, last.time};
}

// ---------------------------------------------------------------------------
// Internal: ternary search for the elevation angle of maximum range.
// Returns {theta_max_deg, max_range_m}.
// ---------------------------------------------------------------------------
std::pair<double, double> find_max_range_angle(
    const TrajectorySimulator& sim,
    double azimuth_deg,
    double muzzle_speed_ms,
    double launch_height_m,
    double lo_deg,
    double hi_deg,
    double target_z_m  = 0.0,
    int    iterations  = 60)
{
    for (int i = 0; i < iterations; ++i) {
        const double span = hi_deg - lo_deg;
        const double m1   = lo_deg + span / 3.0;
        const double m2   = hi_deg - span / 3.0;
        const double r1   = shoot(sim, azimuth_deg, m1, muzzle_speed_ms, launch_height_m, target_z_m).range_m;
        const double r2   = shoot(sim, azimuth_deg, m2, muzzle_speed_ms, launch_height_m, target_z_m).range_m;
        if (r1 < r2) lo_deg = m1;
        else          hi_deg = m2;
    }
    const double theta = (lo_deg + hi_deg) * 0.5;
    const double r_max = shoot(sim, azimuth_deg, theta, muzzle_speed_ms, launch_height_m, target_z_m).range_m;
    return {theta, r_max};
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// solve_elevation
// ---------------------------------------------------------------------------

FireSolution solve_elevation(
    const TrajectorySimulator& sim,
    LauncherOrientation        orientation,
    double                     range_m,
    double                     muzzle_speed_ms,
    double                     launch_height_m,
    bool                       high_angle,
    double                     tolerance_m,
    double                     target_altitude_m)
{
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
    constexpr double kHiSearch =  87.0;

    const double az = orientation.azimuth_deg;

    const bool effective_hi_ang = high_angle;

    auto [theta_max, r_max] =
        find_max_range_angle(sim, az, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch, target_altitude_m);

    if (range_m > r_max) return {};

    double lo = effective_hi_ang ? theta_max : kLoSearch;
    double hi = effective_hi_ang ? kHiSearch : theta_max;

    // Track the best result seen so far (minimum range error).
    // This ensures we return the closest solution even if the bisection
    // exhausts its iteration budget without reaching tolerance_m.
    double best_elev = (lo + hi) * 0.5;
    double best_time = 0.0;
    double best_err  = std::numeric_limits<double>::max();

    for (int i = 0; i < 60; ++i) {
        const double mid  = (lo + hi) * 0.5;
        const auto   shot = shoot(sim, az, mid, muzzle_speed_ms,
                                  launch_height_m, target_altitude_m);

        const double err = std::abs(shot.range_m - range_m);
        if (err < best_err) {
            best_err  = err;
            best_elev = mid;
            best_time = shot.time_s;
        }

        if (err <= tolerance_m) break;

        // Bisection direction:
        //   normal low-angle  : range increases with elevation → standard
        //   high-angle or
        //   ascending mode    : range decreases with elevation → inverted
        if (!effective_hi_ang) {
            if (shot.range_m < range_m) lo = mid; else hi = mid;
        } else {
            if (shot.range_m > range_m) lo = mid; else hi = mid;
        }
    }

    // Guard: if the bisection never brought the range error within 50× the
    // requested tolerance, the target range is not achievable at the given
    // target altitude.  Returning valid=false in this case is correct;
    // callers should use a FireControlTable to find the achievable envelope.
    if (best_err > tolerance_m * 50.0)
        return {};

    return FireSolution{best_elev, best_time * 1000.0, true};
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

InterceptSolution solve_moving_target(
    const TrajectorySimulator& sim,
    const Vec3&                launcher_pos,
    const Vec3&                target_pos,
    const Vec3&                target_velocity,
    double                     muzzle_speed_ms,
    bool                       high_angle,
    double                     tolerance_m,
    int                        max_iterations)
{
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");

    // --- Step 1: initial solve to the current target position ---------------
    const double dx0  = target_pos.x - launcher_pos.x;
    const double dy0  = target_pos.y - launcher_pos.y;
    const double rng0 = std::sqrt(dx0 * dx0 + dy0 * dy0);

    if (rng0 < 1.0) return {}; // launcher and target essentially co-located

    const double az0 = std::atan2(dx0, dy0) * kRadToDeg;
    const double lh0 = launcher_pos.z - target_pos.z;

    LauncherOrientation orient0;
    orient0.azimuth_deg = az0;

    FireSolution fs0 = solve_elevation(sim, orient0, rng0, muzzle_speed_ms,
                                       lh0, high_angle, tolerance_m,
                                       target_pos.z);
    if (!fs0.valid) return {}; // target already out of range

    // Convergence loop
    double flight_time_s = fs0.flight_time_ms / 1000.0;

    FireSolution best_sol = fs0;
    double       best_az  = az0;
    Vec3         best_ipt = target_pos; // intercept point
    int          best_itr = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // --- Step 2: project target position --------------------------------
        const Vec3 ipt = {
            target_pos.x + target_velocity.x * flight_time_s,
            target_pos.y + target_velocity.y * flight_time_s,
            target_pos.z + target_velocity.z * flight_time_s
        };

        // --- Step 3: solve to the projected intercept point -----------------
        const double dx  = ipt.x - launcher_pos.x;
        const double dy  = ipt.y - launcher_pos.y;
        const double rng = std::sqrt(dx * dx + dy * dy);

        if (rng < 1.0) return {}; // intercept collapsed onto launcher

        const double az = std::atan2(dx, dy) * kRadToDeg;
        const double lh = launcher_pos.z - ipt.z;

        LauncherOrientation orient;
        orient.azimuth_deg = az;

        FireSolution fs = solve_elevation(sim, orient, rng, muzzle_speed_ms,
                                          lh, high_angle, tolerance_m, ipt.z);
        if (!fs.valid) return {}; // intercept point is out of range

        const double new_tof = fs.flight_time_ms / 1000.0;

        best_sol = fs;
        best_az  = az;
        best_ipt = ipt;
        best_itr = iter + 1;

        // --- Step 4: check convergence (1 ms threshold) ---------------------
        if (std::abs(new_tof - flight_time_s) < 0.001) break;

        flight_time_s = new_tof;
    }

    // Compute horizontal lead distance
    const double ldx = best_ipt.x - target_pos.x;
    const double ldy = best_ipt.y - target_pos.y;
    const double lead = std::sqrt(ldx * ldx + ldy * ldy);

    InterceptSolution result;
    result.fire           = best_sol;
    result.azimuth_deg    = best_az;
    result.intercept_point = best_ipt;
    result.lead_distance_m = lead;
    result.iterations     = best_itr;
    result.valid          = true;
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

InterceptSolution solve_moving_target_slewed(
    const TrajectorySimulator& sim,
    const Vec3&                launcher_pos,
    double                     current_azimuth_deg,
    double                     current_elevation_deg,
    const Vec3&                target_pos,
    const Vec3&                target_velocity,
    double                     muzzle_speed_ms,
    const LauncherSlew&        slew,
    bool                       high_angle,
    double                     tolerance_m,
    int                        max_iterations)
{
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");

    // Compute the shortest angular distance between two azimuths [0, 180].
    auto az_delta = [](double a, double b) -> double {
        double d = std::abs(a - b);
        return (d > 180.0) ? 360.0 - d : d;
    };

    // Time for the launcher to slew from current orientation to (target_az, target_el).
    auto slew_time_for = [&](double target_az, double target_el) -> double {
        const double t_yaw   = (slew.yaw_deg_per_s   > 0.0)
            ? az_delta(current_azimuth_deg,   target_az)                  / slew.yaw_deg_per_s
            : 0.0;
        const double t_pitch = (slew.pitch_deg_per_s > 0.0)
            ? std::abs(current_elevation_deg - target_el) / slew.pitch_deg_per_s
            : 0.0;
        return std::max(t_yaw, t_pitch);
    };

    double           ts   = 0.0;   // current slew-time estimate (seconds)
    InterceptSolution best;

    for (int iter = 0; iter <= max_iterations; ++iter) {
        // Project target to where it will be when the launcher finishes slewing.
        const Vec3 fire_pos = {
            target_pos.x + target_velocity.x * ts,
            target_pos.y + target_velocity.y * ts,
            target_pos.z + target_velocity.z * ts
        };

        // Solve intercept from that deferred fire position.
        InterceptSolution isol = solve_moving_target(
            sim, launcher_pos, fire_pos, target_velocity,
            muzzle_speed_ms, high_angle, tolerance_m, max_iterations);

        if (!isol.valid) return {};

        // Update slew estimate from the newly computed firing angles.
        const double ts_new = slew_time_for(isol.azimuth_deg,
                                             isol.fire.elevation_deg);
        isol.slew_time_s = ts_new;
        best = isol;

        if (std::abs(ts_new - ts) < 0.01) break;  // converged (10 ms threshold)
        ts = ts_new;
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

void FireControlTable::build(
    const TrajectorySimulator& sim,
    double muzzle_speed_ms,
    double azimuth_deg,
    double launch_height_m,
    bool   high_angle,
    int    num_samples,
    double target_altitude_m)
{
    if (muzzle_speed_ms <= 0.0)
        throw std::invalid_argument("muzzle_speed_ms must be positive");
    if (num_samples < 2) num_samples = 2;

    // Symmetric bounds (see solve_elevation for the rationale behind −87°).
    constexpr double kLoSearch = -87.0;
    constexpr double kHiSearch =  87.0;

    // Use a coarser dt during the sweep — accurate enough for table use
    constexpr double kBuildDt = 1.0 / 120.0;

    const bool effective_hi_ang = high_angle;

    // --- Step 1: quick ternary search for theta_max (20 iterations = 40 sims) ---
    // This concentrates all num_samples in the relevant half of the angle range,
    // giving ~num_samples entries rather than wasting most on the discarded half.
    // Note: uses kBuildDt resolution; a slight theta_max error is tolerated
    // because the monotone filter in step 3 absorbs it.
    auto [theta_max_sb, r_max_unused] =
        find_max_range_angle(sim, azimuth_deg, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch, target_altitude_m,
                             /*iterations=*/20);
    (void)r_max_unused;
    // Copy out of the structured binding into a plain variable.
    // C++17 does not allow lambdas to capture structured bindings directly;
    // that was only standardised in C++20.
    const double theta_max = theta_max_sb;

    // Determine the effective lower bound of the sweep by locating the first
    // angle that yields a positive range.  When the launcher is well above the
    // target every angle gives a positive range and sweep_lo stays at
    // kLoSearch.  When the launcher is at or below the target, angles shallower
    // than the minimum lofting angle produce range=0 (either due to immediate
    // termination or cannot-reach-target-altitude timeout).  Concentrating the
    // samples in the useful band preserves look-up accuracy.
    //
    // For ascending mode (launcher below target) the sweep starts at theta_max
    // (the minimum angle that reaches target altitude) and goes up to 87°,
    // matching the high-angle sweep direction.
    //
    // The binary search costs 20 extra trajectory simulations — negligible
    // versus the num_samples sweep that follows.
    const double sweep_lo = [&]() -> double {
        if (effective_hi_ang) return theta_max;
        double lo_bs = kLoSearch, hi_bs = theta_max;
        for (int i = 0; i < 20; ++i) {
            const double mid_bs = (lo_bs + hi_bs) * 0.5;
            const double r = shoot(sim, azimuth_deg, mid_bs, muzzle_speed_ms,
                                   launch_height_m, target_altitude_m,
                                   kBuildDt).range_m;
            // Narrow toward the first positive-range angle
            if (r > 0.0) hi_bs = mid_bs;
            else         lo_bs = mid_bs;
        }
        // lo_bs is the highest angle still giving range=0; start the sweep
        // just below it so the table captures the full low-range end.
        return lo_bs;
    }();
    const double sweep_hi = effective_hi_ang ? kHiSearch : theta_max;
    const double step     = (sweep_hi - sweep_lo) / (num_samples - 1);

    // --- Step 2: sweep the selected arc ---
    struct Raw { double elev; double range; double tof; };
    std::vector<Raw> raw;
    raw.reserve(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        const double elev = sweep_lo + i * step;
        const auto   s    = shoot(sim, azimuth_deg, elev, muzzle_speed_ms,
                                  launch_height_m, target_altitude_m, kBuildDt);
        raw.push_back({elev, s.range_m, s.time_s});
    }

    // --- Step 3: build entries in ascending range order ---
    // Low-angle         : range increases with elevation → forward pass
    // High-angle or
    // ascending mode    : range decreases with elevation → reverse pass
    entries_.clear();
    entries_.reserve(num_samples);

    if (!effective_hi_ang) {
        double prev = -1.0;
        for (int i = 0; i < num_samples; ++i) {
            if (raw[i].range > prev) {
                entries_.push_back({raw[i].range, raw[i].elev, raw[i].tof * 1000.0});
                prev = raw[i].range;
            }
        }
    } else {
        double prev = -1.0;
        for (int i = num_samples - 1; i >= 0; --i) {
            if (raw[i].range > prev) {
                entries_.push_back({raw[i].range, raw[i].elev, raw[i].tof * 1000.0});
                prev = raw[i].range;
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
    auto it = std::lower_bound(
        entries_.begin(), entries_.end(), range_m,
        [](const Entry& e, double r) { return e.range_m < r; });

    // Below the minimum range stored in the table → no valid solution
    if (it == entries_.begin() && it->range_m != range_m)
        return {};

    // Exact match
    if (it->range_m == range_m)
        return FireSolution{it->elevation_deg, it->flight_time_ms, true};

    // Interpolate between [prev, it]
    const Entry& hi = *it;
    const Entry& lo = *std::prev(it);

    const double span = hi.range_m - lo.range_m;
    // Guard against duplicate range entries (can occur due to floating-point
    // rounding in the sweep).  Return the lower entry directly.
    if (span == 0.0)
        return FireSolution{lo.elevation_deg, lo.flight_time_ms, true};

    const double t = (range_m - lo.range_m) / span;

    return FireSolution{
        lo.elevation_deg  + t * (hi.elevation_deg  - lo.elevation_deg),
        lo.flight_time_ms + t * (hi.flight_time_ms - lo.flight_time_ms),
        true
    };
}

// ---------------------------------------------------------------------------
// FireControlTable::max_range_m
// ---------------------------------------------------------------------------

double FireControlTable::max_range_m() const noexcept {
    return entries_.empty() ? 0.0 : entries_.back().range_m;
}

} // namespace ballistics
