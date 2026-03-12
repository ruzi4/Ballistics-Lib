#pragma once

#include "trajectory.hpp"

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
    double azimuth_deg{0.0};  ///< Horizontal bearing to target (degrees, 0=North, CW)
};

/// Result returned by solve_elevation().
struct FireSolution {
    /// Launch elevation angle above horizontal (degrees).
    /// Negative values indicate a downward shot (launcher above target plane).
    double elevation_deg{0.0};

    /// Time of flight from launch to impact (milliseconds).
    double flight_time_ms{0.0};

    /// True if a solution was found within the munition's maximum range.
    /// False if the target is out of range or the solver did not converge.
    bool valid{false};
};

/// Solve for the launch elevation angle that places the projectile at
/// the given horizontal range, using a ternary search to locate the
/// maximum-range angle followed by bisection to converge on the solution.
///
/// The function accounts for aerodynamic drag and wind (as set on @p sim).
/// Two solutions exist for any achievable range: a low-angle (direct fire)
/// trajectory and a high-angle (plunging fire) trajectory.
///
/// @param sim              Configured TrajectorySimulator (munition + atmosphere)
/// @param orientation      Horizontal bearing from launcher to target
/// @param range_m          Horizontal ground range to target (m)
/// @param muzzle_speed_ms  Muzzle velocity of the projectile (m/s)
/// @param launch_height_m  Height of launcher above the target plane (m).
///                         Use 0 for flat ground; positive when the launcher
///                         is on elevated terrain above the target.
/// @param high_angle       false (default) = low-angle / direct-fire solution
///                         true            = high-angle / plunging-fire solution
/// @param tolerance_m      Convergence threshold: stop bisection when the
///                         simulated range is within this distance of range_m.
///                         Default 0.1 m.
///
/// @returns FireSolution with valid=true on success, valid=false if the
///          target is beyond the maximum range of the munition.
FireSolution solve_elevation(
    const TrajectorySimulator& sim,
    LauncherOrientation        orientation,
    double                     range_m,
    double                     muzzle_speed_ms,
    double                     launch_height_m = 0.0,
    bool                       high_angle      = false,
    double                     tolerance_m     = 0.1);

} // namespace ballistics
