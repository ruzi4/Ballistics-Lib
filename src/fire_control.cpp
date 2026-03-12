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
// The projectile starts at z = launch_height_m and the ground plane is set
// to target_z_m, so the returned range is the horizontal distance from origin
// to the point where the projectile crosses the target elevation.
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
    const double az = azimuth_deg   * kDegToRad;
    const double el = elevation_deg * kDegToRad;

    const Vec3 h_dir{std::sin(az), std::cos(az), 0.0};

    ProjectileState st;
    st.position = Vec3{0.0, 0.0, launch_height_m};
    st.velocity = muzzle_speed_ms * (std::cos(el) * h_dir
                                   + Vec3{0.0, 0.0, std::sin(el)});
    st.time     = 0.0;

    SimulationConfig cfg;
    cfg.dt       = sim_dt;
    cfg.max_time = 60.0;   // 1 minute ceiling (was 600 s — needlessly long)
    cfg.ground_z = target_z_m;
    cfg.use_rk4  = true;

    ProjectileState last = st;
    sim.simulate(st, cfg, [&last](const ProjectileState& s) {
        last = s;
        return true;
    });

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

    // Upper search bound: 87° (3° below vertical).  Near-vertical shots
    // produce degenerate range behaviour under drag (range → 0) so we stop
    // just short of vertical to keep the bisection monotone.
    constexpr double kLoSearch = -45.0;
    constexpr double kHiSearch =  87.0;

    const double az = orientation.azimuth_deg;

    auto [theta_max, r_max] =
        find_max_range_angle(sim, az, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch, target_altitude_m);

    if (range_m > r_max) return {};

    double lo = high_angle ? theta_max : kLoSearch;
    double hi = high_angle ? kHiSearch : theta_max;

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

        if (!high_angle) {
            if (shot.range_m < range_m) lo = mid; else hi = mid;
        } else {
            if (shot.range_m > range_m) lo = mid; else hi = mid;
        }
    }

    return FireSolution{best_elev, best_time * 1000.0, true};
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

    constexpr double kLoSearch = -45.0;
    constexpr double kHiSearch =  87.0;

    // Use a coarser dt during the sweep — accurate enough for table use
    constexpr double kBuildDt = 1.0 / 120.0;

    // --- Step 1: quick ternary search for theta_max (20 iterations = 40 sims) ---
    // This concentrates all num_samples in the relevant half of the angle range,
    // giving ~num_samples entries rather than wasting most on the discarded half.
    // Note: uses kBuildDt resolution; a slight theta_max error is tolerated
    // because the monotone filter in step 3 absorbs it.
    auto [theta_max, r_max_unused] =
        find_max_range_angle(sim, azimuth_deg, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch, target_altitude_m,
                             /*iterations=*/20);
    (void)r_max_unused;

    // The sweep covers only the monotone half that was requested
    const double sweep_lo = high_angle ? theta_max : kLoSearch;
    const double sweep_hi = high_angle ? kHiSearch : theta_max;
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
    // Low-angle  : range increases with elevation → forward pass
    // High-angle : range decreases with elevation → reverse pass
    entries_.clear();
    entries_.reserve(num_samples);

    if (!high_angle) {
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
    // Defensive: build()'s monotone filter guarantees span > 0 for any table
    // produced by this library.  Guard here in case entries_ was populated
    // by external code that did not enforce strict monotonicity.
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
