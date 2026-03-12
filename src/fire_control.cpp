#include "ballistics/fire_control.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace ballistics {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// ---------------------------------------------------------------------------
// Internal: simulate one shot, return (horizontal_range_m, flight_time_s)
// ---------------------------------------------------------------------------
struct ShotResult { double range_m; double time_s; };

ShotResult shoot(
    const TrajectorySimulator& sim,
    double azimuth_deg,
    double elevation_deg,
    double muzzle_speed_ms,
    double launch_height_m,
    double sim_dt = 1.0 / 240.0)
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
    cfg.max_time = 600.0;
    cfg.ground_z = 0.0;
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
    int    iterations = 60)
{
    for (int i = 0; i < iterations; ++i) {
        const double span = hi_deg - lo_deg;
        const double m1   = lo_deg + span / 3.0;
        const double m2   = hi_deg - span / 3.0;
        const double r1   = shoot(sim, azimuth_deg, m1, muzzle_speed_ms, launch_height_m).range_m;
        const double r2   = shoot(sim, azimuth_deg, m2, muzzle_speed_ms, launch_height_m).range_m;
        if (r1 < r2) lo_deg = m1;
        else          hi_deg = m2;
    }
    const double theta = (lo_deg + hi_deg) * 0.5;
    const double r_max = shoot(sim, azimuth_deg, theta, muzzle_speed_ms, launch_height_m).range_m;
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
    double                     tolerance_m)
{
    constexpr double kLoSearch = -45.0;
    constexpr double kHiSearch =  87.0;

    const double az = orientation.azimuth_deg;

    auto [theta_max, r_max] =
        find_max_range_angle(sim, az, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch);

    if (range_m > r_max) return {};

    double lo = high_angle ? theta_max : kLoSearch;
    double hi = high_angle ? kHiSearch : theta_max;

    double result_elev = (lo + hi) * 0.5;
    double result_time = 0.0;

    for (int i = 0; i < 60; ++i) {
        const double mid   = (lo + hi) * 0.5;
        const auto   shot  = shoot(sim, az, mid, muzzle_speed_ms, launch_height_m);

        result_elev = mid;
        result_time = shot.time_s;

        if (std::abs(shot.range_m - range_m) <= tolerance_m) break;

        if (!high_angle) {
            if (shot.range_m < range_m) lo = mid; else hi = mid;
        } else {
            if (shot.range_m > range_m) lo = mid; else hi = mid;
        }
    }

    return FireSolution{result_elev, result_time * 1000.0, true};
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
    int    num_samples)
{
    if (num_samples < 2) num_samples = 2;

    constexpr double kLoSearch = -45.0;
    constexpr double kHiSearch =  87.0;

    // Use a coarser dt during the sweep — accurate enough for table use
    constexpr double kBuildDt = 1.0 / 120.0;

    // --- Step 1: quick ternary search for theta_max (20 iterations = 40 sims) ---
    // This concentrates all num_samples in the relevant half of the angle range,
    // giving ~num_samples entries rather than wasting most on the discarded half.
    auto [theta_max, r_max_unused] =
        find_max_range_angle(sim, azimuth_deg, muzzle_speed_ms, launch_height_m,
                             kLoSearch, kHiSearch, /*iterations=*/20);
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
                                  launch_height_m, kBuildDt);
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

    // Exact match or first entry
    if (it == entries_.begin() || it->range_m == range_m)
        return FireSolution{it->elevation_deg, it->flight_time_ms, true};

    // Interpolate between [prev, it]
    const Entry& hi = *it;
    const Entry& lo = *std::prev(it);

    const double t = (range_m - lo.range_m) / (hi.range_m - lo.range_m);

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
