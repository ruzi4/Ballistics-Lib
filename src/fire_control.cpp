#include "ballistics/fire_control.hpp"
#include <cmath>
#include <utility>

namespace ballistics {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// ---------------------------------------------------------------------------
// Internal: simulate one shot and return (horizontal_range_m, flight_time_s)
// ---------------------------------------------------------------------------
// The simulation uses RK4 at 240 Hz — accurate enough for fire-control work
// and fast enough that the full ternary + bisect search stays well under
// typical latency budgets.
std::pair<double, double> shoot(
    const TrajectorySimulator& sim,
    double                     azimuth_deg,
    double                     elevation_deg,
    double                     muzzle_speed_ms,
    double                     launch_height_m)
{
    const double az = azimuth_deg  * kDegToRad;
    const double el = elevation_deg * kDegToRad;

    // Horizontal unit vector toward target (x=East, y=North)
    const Vec3 h_dir{std::sin(az), std::cos(az), 0.0};

    ProjectileState st;
    st.position = Vec3{0.0, 0.0, launch_height_m};
    st.velocity = muzzle_speed_ms * (std::cos(el) * h_dir
                                   + Vec3{0.0, 0.0, std::sin(el)});
    st.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = 1.0 / 240.0;
    cfg.max_time = 600.0;   // generous upper limit (10 min)
    cfg.ground_z = 0.0;     // target plane
    cfg.use_rk4  = true;

    // Collect only the final (impact) state via streaming callback
    ProjectileState last = st;
    sim.simulate(st, cfg, [&last](const ProjectileState& s) {
        last = s;
        return true;  // always continue — the cfg ground_z stop handles termination
    });

    const double h_range = std::sqrt(last.position.x * last.position.x
                                   + last.position.y * last.position.y);
    return {h_range, last.time};
}

// ---------------------------------------------------------------------------
// Internal: find the elevation angle that maximises horizontal range.
// Uses ternary search on [lo_deg, hi_deg] — valid because range(elevation)
// is strictly unimodal (single maximum) for any combination of drag, wind,
// and non-negative launch height.
// Returns {theta_max_deg, max_range_m}.
// ---------------------------------------------------------------------------
std::pair<double, double> find_max_range_angle(
    const TrajectorySimulator& sim,
    double azimuth_deg,
    double muzzle_speed_ms,
    double launch_height_m,
    double lo_deg,
    double hi_deg,
    int    iterations = 60)
{
    for (int i = 0; i < iterations; ++i) {
        const double span = hi_deg - lo_deg;
        const double m1   = lo_deg + span / 3.0;
        const double m2   = hi_deg - span / 3.0;
        const double r1   = shoot(sim, azimuth_deg, m1, muzzle_speed_ms, launch_height_m).first;
        const double r2   = shoot(sim, azimuth_deg, m2, muzzle_speed_ms, launch_height_m).first;
        if (r1 < r2) lo_deg = m1;
        else          hi_deg = m2;
    }
    const double theta = (lo_deg + hi_deg) * 0.5;
    const double r_max = shoot(sim, azimuth_deg, theta, muzzle_speed_ms, launch_height_m).first;
    return {theta, r_max};
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

FireSolution solve_elevation(
    const TrajectorySimulator& sim,
    LauncherOrientation        orientation,
    double                     range_m,
    double                     muzzle_speed_ms,
    double                     launch_height_m,
    bool                       high_angle,
    double                     tolerance_m)
{
    // Search bounds.
    // Lower bound: -45° handles launchers elevated well above the target.
    // Upper bound: 87° avoids the degenerate vertical case.
    constexpr double kLoSearch = -45.0;
    constexpr double kHiSearch =  87.0;

    const double az = orientation.azimuth_deg;

    // -----------------------------------------------------------------------
    // Step 1 — Locate the maximum-range angle via ternary search
    // -----------------------------------------------------------------------
    auto [theta_max, r_max] =
        find_max_range_angle(sim, az, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch);

    // Target out of range → no solution
    if (range_m > r_max) {
        return {};  // valid = false
    }

    // -----------------------------------------------------------------------
    // Step 2 — Bisect in the appropriate half of the range curve
    //
    //  Low-angle  (direct fire)  : elevation in [kLoSearch, theta_max]
    //                              range is monotonically increasing
    //  High-angle (plunging fire): elevation in [theta_max, kHiSearch]
    //                              range is monotonically decreasing
    // -----------------------------------------------------------------------
    double lo = high_angle ? theta_max  : kLoSearch;
    double hi = high_angle ? kHiSearch  : theta_max;

    double result_elev = (lo + hi) * 0.5;
    double result_time = 0.0;

    for (int i = 0; i < 60; ++i) {
        const double mid = (lo + hi) * 0.5;
        auto [r, t] = shoot(sim, az, mid, muzzle_speed_ms, launch_height_m);

        if (std::abs(r - range_m) <= tolerance_m) {
            result_elev = mid;
            result_time = t;
            break;
        }

        if (!high_angle) {
            // Increasing side: if simulated range < target, need more elevation
            if (r < range_m) lo = mid;
            else             hi = mid;
        } else {
            // Decreasing side: if simulated range > target, need more elevation
            if (r > range_m) lo = mid;
            else             hi = mid;
        }

        result_elev = mid;
        result_time = t;
    }

    return FireSolution{result_elev, result_time * 1000.0, true};
}

} // namespace ballistics
