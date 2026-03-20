#pragma once

#include "fire_control.hpp"
#include "trajectory.hpp"

#include <future>
#include <vector>

namespace ballistics {

// ---------------------------------------------------------------------------
// SolveParams — input for the fire-control solver
// ---------------------------------------------------------------------------
/// All parameters needed to compute a fire solution for a single engagement.
///
/// Supports both stationary and moving targets.  When @c target_moving is
/// false the solver uses solve_elevation(); when true it uses
/// solve_moving_target_slewed() to account for target velocity and launcher
/// rotation time.
struct SolveParams {
    Vec3         launcher_pos;    ///< World-space launcher body centre position (m)
    Vec3         target_pos;      ///< Current target position (m)
    Vec3         target_velocity; ///< Target velocity (m/s); ignored when target_moving is false
    bool         target_moving{false};          ///< True = use moving-target intercept solver
    double       current_azimuth_deg{0.0};      ///< Current physical launcher azimuth (deg)
    double       current_elevation_deg{0.0};    ///< Current physical launcher elevation (deg)
    LauncherSlew slew;                          ///< Launcher yaw/pitch slew rates

    /// Barrel geometry — used to compute the muzzle (barrel tip) position.
    ///
    /// The barrel base is located at launcher_pos + barrel_base_offset_m.
    /// The muzzle is then barrel_length_m along the fire-solution direction
    /// from the barrel base.  Both default to zero, which keeps the trajectory
    /// starting at launcher_pos (backward-compatible behaviour).
    Vec3   barrel_base_offset_m; ///< Offset from launcher body centre to barrel pivot (m)
    double barrel_length_m{0.0}; ///< Physical barrel length from pivot to muzzle (m)

    MunitionSpec munition;                      ///< Projectile specification
    AtmosphericConditions atmosphere;           ///< Atmospheric conditions at the engagement
    double                muzzle_speed_ms{0.0}; ///< Muzzle velocity (m/s)
};

// ---------------------------------------------------------------------------
// SolveResult — output from the fire-control solver
// ---------------------------------------------------------------------------
/// Contains the computed fire solution, trajectory arc, and diagnostic data.
struct SolveResult {
    bool   valid{false};       ///< True if a solution was found
    double azimuth_deg{0.0};   ///< Firing azimuth from launcher to impact (deg, 0=N CW)
    double elevation_deg{0.0}; ///< Launch elevation above horizontal (deg)
    double flight_time_s{0.0}; ///< Time of flight (seconds)
    double range_m{0.0};       ///< Horizontal range to target (m)
    double max_range_m{0.0};   ///< Max achievable range at current settings (m)
    double alt_diff_m{0.0};    ///< Launcher altitude minus target altitude (m)

    /// Trajectory arc in world-space ballistics coordinates (x=East, y=North, z=Up).
    /// Sampled at 60 Hz from muzzle to impact; suitable for visualisation.
    std::vector<Vec3> trajectory;

    /// Muzzle (barrel tip) world-space position computed from launcher_pos,
    /// barrel_base_offset_m, barrel_length_m, and the fire-solution angles.
    /// The trajectory always starts from this point.
    Vec3 muzzle_pos;

    // Moving-target intercept extras
    bool   has_intercept{false}; ///< True when the moving-target solver was used
    Vec3   intercept_point;      ///< Predicted target position at moment of impact (m)
    double lead_distance_m{0.0}; ///< Horizontal lead distance (m)
    double slew_time_s{0.0};     ///< Estimated launcher slew time before shot (s)
    int    intercept_iters{0};   ///< Number of intercept solver iterations
};

// ---------------------------------------------------------------------------
// solve — synchronous fire-control computation
// ---------------------------------------------------------------------------
/// Compute a fire solution for the given engagement parameters.
///
/// This is the heavy computation (~5–100 ms) that should be called from a
/// background thread in a 60 Hz game loop, not from the main render thread.
///
/// The function:
/// 1. Computes the horizontal range and azimuth from launcher to target.
/// 2. Builds a quick diagnostic table to determine maximum range.
/// 3. Solves for elevation using either the static or moving-target solver.
/// 4. Simulates the full trajectory arc for visualisation.
///
/// @param params  Engagement parameters (positions, munition, atmosphere, etc.)
/// @returns       SolveResult with valid=true on success; valid=false if the
///                target is out of range or too close (< 1 m).
SolveResult solve(const SolveParams& params);

// ---------------------------------------------------------------------------
// AsyncSolver — non-blocking fire-control wrapper for game loops
// ---------------------------------------------------------------------------
/// Manages asynchronous fire-control computation on a background thread.
///
/// ## Usage pattern (60 Hz game loop)
/// @code{.cpp}
///   AsyncSolver solver;
///
///   // When parameters change (target moves, weapon swap, etc.):
///   solver.request(params);
///
///   // Every frame:
///   solver.poll();
///   const SolveResult& result = solver.result();
///   if (result.valid) {
///       aim_launcher(result.azimuth_deg, result.elevation_deg);
///       draw_trajectory(result.trajectory);
///   }
/// @endcode
///
/// ## Thread model
///   - request(), poll(), result(), computing() — main thread only.
///   - The heavy solve() runs on a std::async worker thread.
///   - poll() checks completion and swaps in the new result lock-free.
class AsyncSolver {
public:
    AsyncSolver() = default;

    // Non-copyable (owns a std::future)
    AsyncSolver(const AsyncSolver&)            = delete;
    AsyncSolver& operator=(const AsyncSolver&) = delete;
    AsyncSolver(AsyncSolver&&)                 = default;
    AsyncSolver& operator=(AsyncSolver&&)      = default;

    /// Launch a background solve with the given parameters.
    /// If a solve is already in flight, the new request is queued and will
    /// start when the current one completes (checked on the next poll()).
    void request(const SolveParams& params);

    /// Call once per frame.  Checks whether the background solve has completed
    /// and, if so, installs the new result.  Also starts a queued request if
    /// one is pending.
    /// Cost: < 1 µs when no solve is in flight.
    /// @returns true if a new result was installed this call.
    bool poll();

    /// The most recent solve result.  Updated by poll() when a background
    /// computation completes.  Initially returns a default (valid=false) result.
    [[nodiscard]] const SolveResult& result() const noexcept { return current_; }

    /// True while a background solve is running.
    [[nodiscard]] bool computing() const noexcept { return computing_; }

private:
    SolveResult              current_;
    std::future<SolveResult> pending_;
    bool                     computing_{false};
    bool                     queued_{false};
    SolveParams              queued_params_;
};

} // namespace ballistics
