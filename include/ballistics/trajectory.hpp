#pragma once

#include "math/vec3.hpp"
#include "munition.hpp"
#include "atmosphere.hpp"

#include <functional>
#include <vector>

namespace ballistics {

// ---------------------------------------------------------------------------
// Coordinate convention
// ---------------------------------------------------------------------------
// x = East  (m)
// y = North (m)
// z = Up    (m)
// All velocities in m/s, time in seconds.

/// Kinematic state of a projectile at one instant in time.
struct ProjectileState {
    Vec3   position;   ///< World-space position (m)
    Vec3   velocity;   ///< World-space velocity (m/s)
    double time{0.0};  ///< Elapsed simulation time (s)
};

/// Configuration for a full trajectory simulation run.
struct SimulationConfig {
    /// Integration timestep (s).
    /// At 240 Hz (default) the caller's 60 Hz render loop gets 4 physics
    /// sub-steps per frame, balancing accuracy against compute cost.
    double dt{1.0 / 240.0};

    /// Maximum wall-clock simulation time before forced stop (s).
    double max_time{60.0};

    /// Altitude of the ground plane (m). Simulation stops when
    /// projectile.position.z <= ground_z.
    double ground_z{0.0};

    /// Use 4th-order Runge-Kutta (true, default) or symplectic Euler (false).
    /// RK4 is strongly recommended; Euler is provided for benchmarking only.
    bool use_rk4{true};

    // Atmosphere is constant for the entire trajectory.
    // Set AtmosphericConditions on the TrajectorySimulator directly.
};

/// Per-step callback used by the streaming simulate() overload.
/// Return true to continue, false to stop early.
using StepCallback = std::function<bool(const ProjectileState&)>;

// ---------------------------------------------------------------------------
// TrajectorySimulator
// ---------------------------------------------------------------------------

/// Real-time capable ballistic trajectory simulator with aerodynamic drag.
///
/// Physics model
/// -------------
/// Forces acting on the projectile:
///   1. Gravity:     F_g = m * g  (downward, -z)
///   2. Aerodynamic drag:
///        F_d = -½ · Cd · ρ · A · |v|² · v̂
///
/// Equation of motion:
///   a = g_vec − (k · |v|) · v
///   k = ½ · Cd · ρ · A / m        (precomputed drag constant, 1/m)
///
/// Integration is performed with 4th-order Runge-Kutta (RK4) for accuracy,
/// or symplectic Euler when maximum throughput is required.
///
/// Performance notes
/// -----------------
/// A single RK4 step costs ~200 ns on modern hardware (measured on an
/// AMD Ryzen 5 at −O3). At 60 Hz with a 240 Hz sub-step rate the per-frame
/// budget consumed per active projectile is < 15 µs — well under the 16.7 ms
/// frame budget, allowing thousands of concurrent projectiles.
class TrajectorySimulator {
public:
    /// Construct a simulator for @p munition in the given @p atmosphere.
    TrajectorySimulator(const MunitionSpec&         munition,
                        const AtmosphericConditions& atmosphere);

    // -----------------------------------------------------------------------
    // Real-time interface
    // -----------------------------------------------------------------------

    /// Advance @p state forward by @p dt seconds and return the new state.
    /// This is the primary entry point for game / simulation loops.
    ///
    /// @code
    ///   // In your 60 Hz update loop:
    ///   const double sub_dt = 1.0 / 240.0;
    ///   for (int i = 0; i < 4; ++i)
    ///       state = sim.step(state, sub_dt);
    /// @endcode
    [[nodiscard]] ProjectileState step(const ProjectileState& state,
                                       double dt) const noexcept;

    /// Advance @p state forward using RK4 (always, regardless of config).
    [[nodiscard]] ProjectileState step_rk4(const ProjectileState& state,
                                            double dt) const noexcept;

    /// Advance @p state forward using symplectic Euler.
    [[nodiscard]] ProjectileState step_euler(const ProjectileState& state,
                                              double dt) const noexcept;

    // -----------------------------------------------------------------------
    // Batch / offline interface
    // -----------------------------------------------------------------------

    /// Run a complete trajectory from @p initial, storing every step.
    /// @returns  All states from initial to landing (inclusive).
    [[nodiscard]]
    std::vector<ProjectileState> simulate(const ProjectileState& initial,
                                          const SimulationConfig& cfg) const;

    /// Run a complete trajectory, invoking @p callback for each step.
    /// Avoids allocating the full state vector — suitable for real-time
    /// recording or streaming output.
    void simulate(const ProjectileState& initial,
                  const SimulationConfig& cfg,
                  StepCallback callback) const;

    // -----------------------------------------------------------------------
    // Accessors / mutators
    // -----------------------------------------------------------------------

    /// Replace the atmospheric conditions and recompute the drag constant.
    void set_atmosphere(const AtmosphericConditions& atm) noexcept;

    [[nodiscard]] const AtmosphericConditions& atmosphere() const noexcept {
        return atmosphere_;
    }
    [[nodiscard]] const MunitionSpec& munition() const noexcept {
        return munition_;
    }
    /// Precomputed drag constant k = ½·Cd·ρ·A/m  (1/m).
    [[nodiscard]] double drag_k() const noexcept { return drag_k_; }

private:
    MunitionSpec          munition_;
    AtmosphericConditions atmosphere_;
    double                drag_k_{0.0};  ///< Precomputed drag constant

    void update_drag_k() noexcept;

    /// Compute [dpos/dt, dvel/dt] at the given kinematic state.
    void derivatives(const Vec3& pos,
                     const Vec3& vel,
                     Vec3&       dpos_dt,
                     Vec3&       dvel_dt) const noexcept;
};

} // namespace ballistics
