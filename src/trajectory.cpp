#include "ballistics/trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace ballistics {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
namespace {
constexpr double kGravity = -9.80665; // m/s² (negative = downward in z-up)
} // anonymous namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TrajectorySimulator::TrajectorySimulator(const MunitionSpec&          munition,
                                         const AtmosphericConditions& atmosphere)
    : munition_(munition), atmosphere_(atmosphere) {
    update_drag_k();
}

void TrajectorySimulator::update_drag_k() noexcept {
    // k = ½ · Cd · ρ · A_ref / m
    if (munition_.mass_kg > 0.0) {
        drag_k_ = 0.5 * munition_.drag_coefficient * atmosphere_.air_density_kg_m3 *
                  munition_.reference_area_m2 / munition_.mass_kg;
    } else {
        drag_k_ = 0.0;
    }
}

void TrajectorySimulator::set_atmosphere(const AtmosphericConditions& atm) noexcept {
    atmosphere_ = atm;
    update_drag_k();
}

// ---------------------------------------------------------------------------
// Physics kernel
// ---------------------------------------------------------------------------

// Compute derivatives at a given state.
// dpos/dt = vel
// dvel/dt = gravity + drag_acceleration
//   drag_accel = -k · |v_rel| · v_rel   where v_rel = vel − wind
void TrajectorySimulator::derivatives(const Vec3& /*pos*/,
                                      const Vec3& vel,
                                      double      air_density_kg_m3,
                                      Vec3&       dpos_dt,
                                      Vec3&       dvel_dt) const noexcept {
    dpos_dt = vel;

    // Fast path: skip drag computation entirely for vacuum projectiles (Cd=0).
    // Avoids the wind subtraction, norm_sq, sqrt, and three multiplications
    // per call (= 4 calls per RK4 step, so 16 fewer FP ops per step).
    if (drag_k_ == 0.0) {
        dvel_dt = Vec3{0.0, 0.0, kGravity};
        return;
    }

    // Velocity relative to the wind
    const Vec3   relative_velocity_ms    = vel - atmosphere_.wind.velocity_ms;
    const double relative_speed_sq_m2s2  = relative_velocity_ms.norm_sq();

    // Drag: a_drag = -k_rho · |v_rel| · v_rel
    // Scale the precomputed drag_k_ (which uses the fixed atmosphere density)
    // by the ratio air_density / reference_density.  This avoids re-reading
    // three munition fields and saves 2 multiplications per derivatives() call
    // (= 8 per RK4 step).  Falls back to zero when reference_density == 0 (vacuum).
    const double reference_density_kg_m3  = atmosphere_.air_density_kg_m3;
    const double drag_constant_scaled_1_per_m = (reference_density_kg_m3 > 0.0)
                                                    ? drag_k_ * (air_density_kg_m3 / reference_density_kg_m3)
                                                    : 0.0;

    const double relative_speed_ms    = std::sqrt(relative_speed_sq_m2s2);
    const Vec3   drag_acceleration_ms2 = -(drag_constant_scaled_1_per_m * relative_speed_ms) * relative_velocity_ms;

    dvel_dt = Vec3{0.0, 0.0, kGravity} + drag_acceleration_ms2;
}

// ---------------------------------------------------------------------------
// Single-step integrators
// ---------------------------------------------------------------------------

ProjectileState
TrajectorySimulator::step_rk4_rho(const ProjectileState& s, double dt, double air_density_kg_m3) const noexcept {
    Vec3 dpos_dt, dvel_dt;

    // k1
    derivatives(s.position, s.velocity, air_density_kg_m3, dpos_dt, dvel_dt);
    const Vec3 k1_p = dpos_dt;
    const Vec3 k1_v = dvel_dt;

    // k2
    derivatives(s.position + 0.5 * dt * k1_p, s.velocity + 0.5 * dt * k1_v, air_density_kg_m3, dpos_dt, dvel_dt);
    const Vec3 k2_p = dpos_dt;
    const Vec3 k2_v = dvel_dt;

    // k3
    derivatives(s.position + 0.5 * dt * k2_p, s.velocity + 0.5 * dt * k2_v, air_density_kg_m3, dpos_dt, dvel_dt);
    const Vec3 k3_p = dpos_dt;
    const Vec3 k3_v = dvel_dt;

    // k4
    derivatives(s.position + dt * k3_p, s.velocity + dt * k3_v, air_density_kg_m3, dpos_dt, dvel_dt);
    const Vec3      k4_p = dpos_dt;
    const Vec3      k4_v = dvel_dt;

    const double    dt_sixth = dt / 6.0;
    ProjectileState next;
    next.position = s.position + dt_sixth * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
    next.velocity = s.velocity + dt_sixth * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
    next.time     = s.time + dt;
    return next;
}

ProjectileState TrajectorySimulator::step_euler_rho(const ProjectileState& s,
                                                    double                 dt,
                                                    double                 air_density_kg_m3) const noexcept {
    Vec3 dpos_dt, dvel_dt;
    derivatives(s.position, s.velocity, air_density_kg_m3, dpos_dt, dvel_dt);

    ProjectileState next;
    // Symplectic Euler: update velocity first, then position with new velocity
    next.velocity = s.velocity + dt * dvel_dt;
    next.position = s.position + dt * next.velocity;
    next.time     = s.time + dt;
    return next;
}

ProjectileState TrajectorySimulator::step_rk4(const ProjectileState& s, double dt) const noexcept {
    return step_rk4_rho(s, dt, atmosphere_.air_density_kg_m3);
}

ProjectileState TrajectorySimulator::step_euler(const ProjectileState& s,
                                                double                 dt) const noexcept {
    return step_euler_rho(s, dt, atmosphere_.air_density_kg_m3);
}

ProjectileState TrajectorySimulator::step(const ProjectileState& state, double dt) const noexcept {
    return step_rk4(state, dt);
}

// ---------------------------------------------------------------------------
// Batch simulation helpers
// ---------------------------------------------------------------------------

// Internal core used by both batch overloads.
// Calls step_fn(state, dt) → ProjectileState and notifies callback.
void TrajectorySimulator::simulate(const ProjectileState&  initial,
                                   const SimulationConfig& cfg,
                                   StepCallback            callback) const {
    if (cfg.dt <= 0.0)
        throw std::invalid_argument("SimulationConfig::dt must be positive");

    ProjectileState state = initial;

    // If the projectile starts at or below the ground plane and has no upward
    // velocity (e.g. negative elevation with zero launch height), it cannot
    // become airborne — report immediate impact and return.
    if (state.position.z <= cfg.ground_z && state.velocity.z <= 0.0) {
        state.position.z = cfg.ground_z;
        callback(state);
        return;
    }

    // Notify caller of the initial state
    if (!callback(state))
        return;

    while (state.time < cfg.max_time) {
        double dt = cfg.dt;
        // Don't overshoot max_time
        if (state.time + dt > cfg.max_time)
            dt = cfg.max_time - state.time;
        if (dt <= 0.0)
            break;

        // Resolve air density for this step (altitude-varying if requested).
        // Wind is always taken from the simulator's stored atmosphere.
        const double step_air_density_kg_m3 = cfg.atmosphere_fn
                                                  ? cfg.atmosphere_fn(state.position.z).air_density_kg_m3
                                                  : atmosphere_.air_density_kg_m3;

        ProjectileState next;
        if (cfg.use_rk4) {
            next = step_rk4_rho(state, dt, step_air_density_kg_m3);
        } else {
            next = step_euler_rho(state, dt, step_air_density_kg_m3);
        }

        // Ground intersection: linearly interpolate to the exact crossing time
        if (next.position.z <= cfg.ground_z && state.position.z > cfg.ground_z) {
            const double    altitude_delta_m      = state.position.z - next.position.z;
            const double    ground_crossing_frac  = (altitude_delta_m > 0.0)
                                                        ? (state.position.z - cfg.ground_z) / altitude_delta_m
                                                        : 0.0;
            ProjectileState impact;
            impact.position   = state.position + ground_crossing_frac * (next.position - state.position);
            impact.velocity   = state.velocity + ground_crossing_frac * (next.velocity - state.velocity);
            impact.time       = state.time + ground_crossing_frac * dt;
            impact.position.z = cfg.ground_z; // clamp exactly to ground
            callback(impact);
            return;
        }

        state = next;
        if (!callback(state))
            return;
    }
}

std::vector<ProjectileState> TrajectorySimulator::simulate(const ProjectileState&  initial,
                                                           const SimulationConfig& cfg) const {
    std::vector<ProjectileState> states;
    // Pre-allocate a reasonable amount to avoid repeated reallocation
    const double estimated_steps = cfg.max_time / cfg.dt;
    states.reserve(static_cast<std::size_t>(estimated_steps) + 2);

    simulate(initial, cfg, [&states](const ProjectileState& s) -> bool {
        states.push_back(s);
        return true;
    });

    return states;
}

} // namespace ballistics
