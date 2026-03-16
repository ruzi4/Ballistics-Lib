#pragma once

#include "trajectory.hpp"

#include <vector>

namespace ballistics {

// ---------------------------------------------------------------------------
// Coordinate convention (matches the rest of the library)
// ---------------------------------------------------------------------------
// x = East, y = North, z = Up
// Azimuth is measured from North, clockwise:
//   0°  = North (+y)
//   90° = East  (+x)
//  180° = South (-y)
//  270° = West  (-x)

/// Horizontal orientation of the launcher (azimuth only).
/// Elevation is the output of solve_elevation(), not an input.
struct LauncherOrientation {
    double azimuth_deg{0.0}; ///< Horizontal bearing to target (degrees, 0=North, CW)
};

/// Physical slew (rotation) rates of the launcher mechanism.
/// Used by solve_moving_target_slewed() to account for the time needed to
/// rotate from the current orientation to the required firing angle before
/// the shot can be released.
struct LauncherSlew {
    double yaw_deg_per_s{25.0};   ///< Azimuth (yaw) slew rate (deg/s)
    double pitch_deg_per_s{25.0}; ///< Elevation (pitch) slew rate (deg/s)
};

/// Result returned by solve_elevation() and FireControlTable::lookup().
struct FireSolution {
    /// Launch elevation angle above horizontal (degrees).
    /// Negative values indicate a downward shot (launcher above target plane).
    double elevation_deg{0.0};

    /// Time of flight from launch to impact (milliseconds).
    double flight_time_ms{0.0};

    /// True if a solution was found within the munition's maximum range.
    bool valid{false};
};

/// Result of a moving-target intercept computation.
/// solve_moving_target() iteratively forward-projects the target's position
/// by the estimated time of flight until the intercept point converges.
struct InterceptSolution {
    /// Elevation and flight-time solution aimed at the intercept point.
    FireSolution fire;

    /// Horizontal bearing from launcher to intercept point (degrees, 0=North, CW).
    double azimuth_deg{0.0};

    /// Predicted world-space position of the target at the moment of impact (m).
    Vec3 intercept_point{};

    /// Horizontal distance between the current target position and the
    /// intercept point (i.e. how far ahead of the target we are aiming) (m).
    double lead_distance_m{0.0};

    /// Estimated time for the launcher to slew from its current orientation
    /// to the required firing angle (seconds).  Zero when returned by
    /// solve_moving_target(); populated by solve_moving_target_slewed().
    double slew_time_s{0.0};

    /// Number of fixed-point iterations used to converge.
    int iterations{0};

    /// True if a valid intercept solution was found.
    bool valid{false};
};

// ---------------------------------------------------------------------------
// solve_elevation — accurate but NOT real-time
// ---------------------------------------------------------------------------
/// Solve for the launch elevation angle that places the projectile at
/// the given horizontal range using ternary search + bisection.
///
/// @note  Cost: ~120–180 full trajectory simulations per call.
///        Typical wall time: 5–100 ms depending on trajectory length.
///        Use FireControlTable::lookup() for per-frame game queries.
///
/// @param sim                Configured TrajectorySimulator (munition + atmosphere)
/// @param orientation        Horizontal bearing from launcher to target
/// @param range_m            Horizontal ground range to target (m)
/// @param muzzle_speed_ms    Muzzle velocity (m/s)
/// @param launch_height_m    Height of launcher above the target plane (m)
/// @param high_angle         false = low-angle (direct fire), true = high-angle (plunging fire)
/// @param tolerance_m        Bisection convergence threshold (m)  [default 0.1]
/// @param target_altitude_m  Altitude of the target / ground plane (m) [default 0.0].
///                           Set to a non-zero value when the target is at a
///                           different elevation than z = 0.
///
/// @returns FireSolution with valid=true on success, valid=false if
///          @p range_m exceeds the munition's maximum range.
///
/// @throws std::invalid_argument if @p muzzle_speed_ms <= 0.
FireSolution solve_elevation(const TrajectorySimulator& sim,
                             LauncherOrientation        orientation,
                             double                     range_m,
                             double                     muzzle_speed_ms,
                             double                     launch_height_m   = 0.0,
                             bool                       high_angle        = false,
                             double                     tolerance_m       = 0.1,
                             double                     target_altitude_m = 0.0);

// ---------------------------------------------------------------------------
// solve_moving_target — intercept fire solution for a moving target
// ---------------------------------------------------------------------------
/// Compute the launch azimuth and elevation required to intercept a moving
/// target, accounting for its velocity at the time of impact.
///
/// ### Algorithm (iterative fixed-point)
/// 1. Estimate the time of flight @c T to the target's *current* position.
/// 2. Forward-project the target: @c intercept = target_pos + target_vel * T.
/// 3. Re-solve the fire solution to the projected intercept position,
///    obtaining a new flight time @c T'.
/// 4. Repeat from step 2 with @c T = T' until |T' − T| < 1 ms, or until
///    @p max_iterations is reached.
///
/// The algorithm converges in 2–5 iterations for typical engagements
/// (subsonic targets at direct-fire ranges).  Supersonic or extreme-range
/// targets may require more iterations or may not converge at all, in which
/// case valid=false is returned.
///
/// @note  Cost: (max_iterations + 1) × solve_elevation() calls, each of
///        which costs ~120–180 trajectory simulations.  Intended to be called
///        from a background thread, not the main render loop.
///
/// @param sim             Simulator configured with the desired munition and
///                        atmospheric conditions.
/// @param launcher_pos    World-space position of the launcher (m).
/// @param target_pos      Current world-space position of the target (m).
/// @param target_velocity Velocity of the target (m/s, ballistics coords:
///                        x=East, y=North, z=Up).
/// @param muzzle_speed_ms Muzzle velocity (m/s).
/// @param high_angle      false = direct fire, true = plunging fire.
/// @param tolerance_m     Range convergence threshold for each inner
///                        solve_elevation() call (m).  [default 0.5]
/// @param max_iterations  Maximum fixed-point outer iterations.  [default 10]
///
/// @returns InterceptSolution with valid=true if an intercept solution was
///          found; valid=false if the target is out of range or the iteration
///          did not converge.
InterceptSolution solve_moving_target(const TrajectorySimulator& sim,
                                      const Vec3&                launcher_pos,
                                      const Vec3&                target_pos,
                                      const Vec3&                target_velocity,
                                      double                     muzzle_speed_ms,
                                      bool                       high_angle     = false,
                                      double                     tolerance_m    = 0.5,
                                      int                        max_iterations = 10);

// ---------------------------------------------------------------------------
// solve_moving_target_slewed — intercept accounting for launcher slew time
// ---------------------------------------------------------------------------
/// Slew-aware extension of solve_moving_target().
///
/// A physical launcher takes time to rotate from its current orientation to
/// the required firing angle.  During that slew period the target continues
/// to move, so the true intercept point is further ahead than a pure
/// flight-time lead calculation would suggest.
///
/// ### Algorithm (outer fixed-point wrapping solve_moving_target)
/// 1. Start with a slew-time estimate T_s = 0.
/// 2. Forward-project the target by T_s to get the "fire point" — where the
///    target will be at the moment the shot can be released.
/// 3. Call solve_moving_target() from that fire point to compute the optimal
///    intercept and obtain updated firing angles (az, el).
/// 4. Recompute T_s = max(|Δaz| / yaw_rate, |Δel| / pitch_rate).
/// 5. Repeat from step 2 until |ΔT_s| < 10 ms or @p max_iterations.
///
/// The result's @c slew_time_s field carries the converged slew estimate.
/// The @c intercept_point is the target's predicted position at the moment
/// of impact (fire point + flight-time lead).
///
/// @note  Cost: (max_iterations + 1) × solve_moving_target() calls.
///        Intended for background-thread use alongside the main render loop.
///
/// @param sim                   Simulator (munition + atmosphere).
/// @param launcher_pos          World-space launcher position (m).
/// @param current_azimuth_deg   Current physical azimuth of the launcher (deg).
/// @param current_elevation_deg Current physical elevation of the launcher (deg).
/// @param target_pos            Current target position (m).
/// @param target_velocity       Target velocity (m/s, x=East, y=North, z=Up).
/// @param muzzle_speed_ms       Muzzle velocity (m/s).
/// @param slew                  Launcher yaw/pitch slew rates.
/// @param high_angle            false = direct fire, true = plunging fire.
/// @param tolerance_m           Inner solve_elevation() convergence threshold (m).
/// @param max_iterations        Outer fixed-point iteration limit.
InterceptSolution solve_moving_target_slewed(const TrajectorySimulator& sim,
                                             const Vec3&                launcher_pos,
                                             double                     current_azimuth_deg,
                                             double                     current_elevation_deg,
                                             const Vec3&                target_pos,
                                             const Vec3&                target_velocity,
                                             double                     muzzle_speed_ms,
                                             const LauncherSlew&        slew,
                                             bool                       high_angle     = false,
                                             double                     tolerance_m    = 0.5,
                                             int                        max_iterations = 10);

// ---------------------------------------------------------------------------
// FireControlTable — O(log N) real-time lookup
// ---------------------------------------------------------------------------
/// Pre-computed range → (elevation, flight-time) look-up table optimised
/// for use inside a 60 Hz game loop.
///
/// ## Usage pattern
/// @code{.cpp}
/// // --- At startup / weapon change / weather change ---
/// FireControlTable table;
/// table.build(sim, muzzle_speed_ms);   // ~50–300 ms, run async if needed
///
/// // --- Every frame (microseconds) ---
/// FireSolution sol = table.lookup(range_to_target_m);
/// if (sol.valid)
///     aim_launcher(sol.elevation_deg);
/// @endcode
///
/// ## Performance
/// build()  — sweeps @p num_samples elevation angles, one simulation each.
///            Typical cost: 50–300 ms (depends on trajectory length).
///            Run on a background thread or during a loading screen.
///
/// lookup() — binary search + linear interpolation: O(log N), < 1 µs.
///            Safe to call from the main render/update thread every frame.
///
/// ## Accuracy
/// Interpolation error is proportional to range / num_samples.
/// With the default 500 samples and max_range ~2 km, angular error is
/// typically < 0.01°.  Increase num_samples for longer ranges.
class FireControlTable {
public:
    /// A single entry in the pre-computed table.
    struct Entry {
        double range_m;        ///< Horizontal range (m)
        double elevation_deg;  ///< Required launch elevation (degrees)
        double flight_time_ms; ///< Time of flight (ms)
    };

    // -------------------------------------------------------------------
    // Building the table
    // -------------------------------------------------------------------

    /// Sweep @p num_samples elevation angles, simulate each trajectory,
    /// and build a range-sorted look-up table.
    ///
    /// @note  Not thread-safe: build() must not be called concurrently with
    ///        lookup(), ready(), or another build() on the same table instance.
    ///        Run build() on a background thread only if the table object is
    ///        not yet shared with other threads.
    ///
    /// @param sim               Simulator configured with the desired munition
    ///                          and atmospheric conditions.
    /// @param muzzle_speed_ms   Muzzle velocity (m/s).
    /// @param azimuth_deg       Horizontal bearing used during the sweep (degrees).
    ///                          Irrelevant in still air; set to the expected
    ///                          firing direction when wind is present.
    /// @param launch_height_m   Launcher height above the target plane (m).
    /// @param high_angle        false = direct-fire table (elevation < theta_max)
    ///                          true  = plunging-fire table (elevation > theta_max)
    /// @param num_samples       Number of elevation samples to sweep.
    /// @param target_altitude_m Altitude of the target / ground plane (m).
    ///                          Defaults to 0.  Set to a non-zero value when
    ///                          firing at an elevated or depressed target.
    ///
    /// @throws std::invalid_argument if @p muzzle_speed_ms <= 0.
    void build(const TrajectorySimulator& sim,
               double                     muzzle_speed_ms,
               double                     azimuth_deg       = 0.0,
               double                     launch_height_m   = 0.0,
               bool                       high_angle        = false,
               int                        num_samples       = 500,
               double                     target_altitude_m = 0.0);

    // -------------------------------------------------------------------
    // Real-time lookup
    // -------------------------------------------------------------------

    /// Return an interpolated FireSolution for @p range_m.
    /// O(log N) — safe to call every frame.
    ///
    /// @returns  valid=false when any of the following are true:
    ///           - The table has not been built yet (call ready() to check).
    ///           - @p range_m is negative.
    ///           - @p range_m is below the table's minimum stored range.
    ///           - @p range_m exceeds the table's maximum range.
    [[nodiscard]] FireSolution lookup(double range_m) const;

    // -------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------

    /// Maximum achievable range stored in this table (m).
    /// Returns 0 if the table is empty.
    [[nodiscard]] double max_range_m() const noexcept;

    /// True after a successful build().
    [[nodiscard]] bool ready() const noexcept { return !entries_.empty(); }

    /// Raw sorted entries — useful for debug visualisation or export.
    [[nodiscard]] const std::vector<Entry>& entries() const noexcept { return entries_; }

private:
    std::vector<Entry> entries_; // sorted by range_m ascending
};

} // namespace ballistics
