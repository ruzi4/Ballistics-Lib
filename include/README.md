# include/

Public C++17 headers for Ballistics-Lib, located under `include/ballistics/`.

Include the umbrella header to pull in the entire API at once:

```cpp
#include <ballistics/ballistics.hpp>
```

Or include individual headers as needed.

---

## Header overview

### `ballistics/ballistics.hpp`
Umbrella header. Includes all of the headers below.

---

### `ballistics/trajectory.hpp`
Core simulation API.

- **`ProjectileState`** — position (`Vec3`, m), velocity (`Vec3`, m/s), time (s)
- **`SimulationConfig`** — timestep (`dt`), max simulation time, ground altitude
  (`ground_z`), integrator choice (`use_rk4`), and an optional
  `atmosphere_fn` callback for altitude-varying air density per step
- **`StepCallback`** — `std::function<bool(const ProjectileState&)>`; return
  `false` to stop simulation early
- **`TrajectorySimulator`** — constructed from a `MunitionSpec` and
  `AtmosphericConditions`; exposes:
  - `step(state, dt)` — single RK4 integration step (~200 ns); primary entry
    point for 60 Hz game loops
  - `step_rk4(state, dt)` — explicit RK4 step (ignores config integrator choice)
  - `step_euler(state, dt)` — explicit symplectic-Euler step
  - `simulate(initial, cfg)` — batch run; returns all `ProjectileState` values
    from launch to ground impact
  - `simulate(initial, cfg, callback)` — streaming variant; invokes `callback`
    at each step; avoids allocating the full state vector
  - `set_atmosphere(atm)` — replace atmospheric conditions and recompute the
    precomputed drag constant `k = ½·Cd·ρ·A/m`

**Physics model:**
```
F_drag = −½ · Cd · ρ · A · |v_rel|² · v̂_rel    (v_rel = v_projectile − v_wind)
a      = g_vec + F_drag / m
g      = −9.80665 m/s²  (z-down component)
```

---

### `ballistics/fire_control.hpp`
Targeting and pre-computed range tables.

**Types:**
- **`LauncherOrientation`** — azimuth in degrees (0 = North, 90 = East)
- **`FireSolution`** — `elevation_deg`, `flight_time_ms`, `valid` flag
- **`InterceptSolution`** — wraps a `FireSolution` with additional fields:
  `azimuth_deg`, `intercept_point` (`Vec3`), `lead_distance_m`,
  `slew_time_s`, `iterations`, `valid`
- **`LauncherSlew`** — `yaw_deg_per_s` and `pitch_deg_per_s` rotation rates
  for a physical launcher

**Free functions:**

`solve_elevation(sim, orientation, range_m, muzzle_speed_ms, ...)` → `FireSolution`
- Ternary-search + bisection; runs 120–180 full trajectory simulations
- Typical cost: 5–100 ms — intended for async / background-thread use
- Supports `launch_height_m` (launcher above/below target plane),
  `high_angle` (plunging fire), and `target_altitude_m`

`solve_moving_target(sim, launcher_pos, target_pos, target_velocity, muzzle_speed_ms, ...)` → `InterceptSolution`
- Iterative fixed-point: estimates flight time T, forward-projects target by
  T, re-solves elevation to the projected position, repeats until |ΔT| < 1 ms
- Converges in 2–5 iterations for typical subsonic targets at direct-fire ranges
- Cost: (max_iterations + 1) × `solve_elevation()` — background-thread use only

`solve_moving_target_slewed(sim, launcher_pos, current_az, current_el, target_pos, target_velocity, muzzle_speed_ms, slew, ...)` → `InterceptSolution`
- Extends `solve_moving_target` by accounting for the time the launcher takes
  to physically rotate from its current orientation to the required firing angle
- Outer fixed-point: estimates slew time T_s, defers target forward-projection
  by T_s, re-solves intercept, recomputes T_s until |ΔT_s| < 10 ms
- Result's `slew_time_s` carries the converged slew estimate;
  `intercept_point` is the target's predicted position at moment of impact

**`FireControlTable`** — O(log N) real-time lookup table:
- `build(sim, muzzle_speed_ms, ...)` — sweeps `num_samples` elevation angles
  (default 500), simulates each; typical cost 50–300 ms; run async or at startup
- `lookup(range_m)` → `FireSolution` — binary search + linear interpolation;
  < 1 µs per call; safe to call from the render thread every frame
- `ready()`, `max_range_m()`, `entries()` — state accessors
- Accuracy: with 500 samples at ~2 km max range, angular error < 0.01°;
  lookup agrees with `solve_elevation` within 0.2° (verified by tests)

---

### `ballistics/async_solver.hpp`
Non-blocking fire-control solver for game loops.

- **`SolveParams`** — all inputs for a single engagement: `launcher_pos`,
  `target_pos`, `target_velocity`, `target_moving` flag,
  `current_azimuth_deg`, `current_elevation_deg`, `slew` rates,
  `munition`, `atmosphere`, `muzzle_speed_ms`
- **`SolveResult`** — full solver output: `valid`, `azimuth_deg`,
  `elevation_deg`, `flight_time_s`, `range_m`, `max_range_m`, `alt_diff_m`,
  `trajectory` (`std::vector<Vec3>` in world coords), plus moving-target
  fields: `has_intercept`, `intercept_point`, `lead_distance_m`,
  `slew_time_s`, `intercept_iters`
- **`solve(params)`** → `SolveResult` — synchronous solver; computes
  diagnostic max-range table, dispatches to `solve_elevation` or
  `solve_moving_target_slewed`, and generates the trajectory arc.
  Cost: 5–500 ms — intended for background-thread use.
- **`AsyncSolver`** — non-blocking wrapper for 60 Hz game loops:
  - `request(params)` — launch a background solve; queues if one is in flight
  - `poll()` — call once per frame; installs completed results and starts
    queued requests
  - `result()` → `const SolveResult&` — latest result (default `valid=false`
    until the first solve completes)
  - `computing()` — true while a background solve is running

---

### `ballistics/munition.hpp`
Projectile specification.

- **`MunitionSpec`** — `name`, `mass_kg`, `density_kg_m3`, `reference_area_m2`,
  `drag_coefficient` (0.0 = vacuum sentinel), `muzzle_velocity_ms`,
  `diameter_m`; derived methods:
  - `ballistic_coefficient()` → `mass / (Cd × A_ref)` kg/m²
  - `volume_m3()` → `mass / density` m³
- **`MunitionLibrary`** — in-memory store for munitions loaded from JSON
  - `load(path)` — load from file; merges into existing library
  - `load_from_string(json)` — load from an in-memory JSON string
  - `get(name)` → `const MunitionSpec&` — throws `std::out_of_range` if not found
  - `contains(name)`, `names()`, `size()`, `clear()`

---

### `ballistics/atmosphere.hpp`
Atmospheric conditions and models.

- **`Wind`** — `velocity_ms` (`Vec3`, m/s in world space: x=East, y=North, z=Up)
- **`AtmosphericConditions`** — `temperature_K`, `pressure_Pa`,
  `relative_humidity` [0,1], `air_density_kg_m3` (pre-computed), `wind`
  - `recompute_density()` — recomputes `air_density_kg_m3` from T, P, RH
- **`compute_air_density(T_K, P_Pa, RH)`** — ideal gas law with Magnus-formula
  water-vapour pressure correction
- **`isa_conditions(altitude_m, ...)`** — ISA model covering troposphere
  (0–11 km, linear temperature lapse) and lower stratosphere (11–20 km,
  isothermal at 216.65 K); clamped to stratopause values above 20 km

---

### `ballistics/math/vec3.hpp`
Double-precision 3D vector (`Vec3`) with fully inlined arithmetic, dot/cross
products, norm, and normalisation.

### `ballistics/math/math_constants.hpp`
`kDegToRad` and `kRadToDeg` conversion constants.
