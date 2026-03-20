# Ballistics-Lib Architecture and Design

## Overview

Ballistics-Lib is a C++17 library for real-time projectile trajectory simulation and fire-control computation. It models the full flight of a projectile under gravity, aerodynamic drag, wind, and altitude-varying atmospheric density, and provides solvers to compute firing angles for both static and moving targets.

---

## Directory Structure

```
Ballistics-Lib/
├── include/ballistics/          # Public API headers
│   ├── ballistics.hpp           # Umbrella include
│   ├── trajectory.hpp           # Core trajectory simulator
│   ├── fire_control.hpp         # Elevation and intercept solvers
│   ├── async_solver.hpp         # Non-blocking solver wrapper
│   ├── atmosphere.hpp           # Atmospheric model
│   ├── munition.hpp             # Projectile specification
│   └── math/
│       ├── vec3.hpp             # 3D vector math
│       └── math_constants.hpp   # Unit conversion constants
├── src/                         # Implementation files
│   ├── trajectory.cpp
│   ├── fire_control.cpp
│   ├── async_solver.cpp
│   ├── atmosphere.cpp
│   └── munition.cpp
├── data/
│   ├── munitions.json           # Pre-configured projectile specs
│   └── launcher_config.json     # Default launcher slew rates
├── examples/                    # Usage demonstrations
└── tests/                       # Unit tests
```

---

## Primary Data Structures

### `MunitionSpec` (`include/ballistics/munition.hpp`)

Describes the physical properties of a projectile.

| Field | Type | Description |
|---|---|---|
| `name` | `std::string` | Unique identifier |
| `mass_kg` | `double` | Projectile mass (kg) |
| `density_kg_m3` | `double` | Material density (kg/m³) |
| `reference_area_m2` | `double` | Cross-sectional reference area (m²) |
| `drag_coefficient` | `double` | Dimensionless aerodynamic drag coefficient (Cd); 0 = vacuum |
| `muzzle_velocity_ms` | `double` | Muzzle velocity (m/s) |
| `diameter_m` | `double` | Characteristic diameter (m, optional) |

Computed property:

```
ballistic_coefficient = mass / (Cd × A_ref)  [kg/m²]
```

A higher ballistic coefficient means the projectile retains velocity better over distance — it decelerates more slowly against drag.

Five reference projectiles are provided in `data/munitions.json`:

| Name | Mass | Cd | Muzzle Velocity |
|---|---|---|---|
| `9mm_fmj_115gr` | 7.45 g | 0.295 | 370 m/s |
| `5.56x45_m855_62gr` | 4.02 g | 0.307 | 930 m/s |
| `7.62x51_m80_147gr` | 9.53 g | 0.295 | 853 m/s |
| `338_lapua_250gr` | 16.2 g | 0.230 | 905 m/s |
| `50_bmg_660gr` | 42.8 g | 0.620 | 928 m/s |

---

### `AtmosphericConditions` (`include/ballistics/atmosphere.hpp`)

Captures the ambient environment the projectile travels through.

| Field | Type | Description |
|---|---|---|
| `temperature_K` | `double` | Air temperature (Kelvin) |
| `pressure_Pa` | `double` | Absolute pressure (Pascal) |
| `relative_humidity` | `double` | Fractional humidity [0, 1] |
| `air_density_kg_m3` | `double` | Pre-computed air density (kg/m³) |
| `wind` | `Wind` | Ambient wind |

**`Wind`**

| Field | Type | Description |
|---|---|---|
| `velocity_ms` | `Vec3` | World-space wind vector (m/s) |

Air density is computed using a moist-air model. The Magnus formula provides the saturation vapor pressure, which is then used to split the total pressure into dry and moist components:

```
P_sat   = 611.657 · exp(17.368·(T − 273.15) / (T − 273.15 + 238.88))
P_vapor = relative_humidity · P_sat
P_dry   = P_total − P_vapor

ρ = (P_dry·M_dry + P_vapor·M_water) / (R·T)
```

---

### `ProjectileState` (`include/ballistics/trajectory.hpp`)

A snapshot of the projectile at a single instant in time.

| Field | Type | Description |
|---|---|---|
| `position` | `Vec3` | World-space position (m), Z = up |
| `velocity` | `Vec3` | World-space velocity (m/s) |
| `time` | `double` | Elapsed simulation time (s) |

---

### `SimulationConfig` (`include/ballistics/trajectory.hpp`)

Controls the numerical integration behaviour.

| Field | Type | Default | Description |
|---|---|---|---|
| `dt` | `double` | 1/240 s | Integration timestep |
| `max_time` | `double` | 60 s | Hard simulation time limit |
| `ground_z` | `double` | 0 m | Ground-plane altitude (Z) |
| `use_rk4` | `bool` | `true` | Use RK4 (true) or symplectic Euler (false) |
| `atmosphere_fn` | `function<AtmosphericConditions(double)>` | `nullptr` | Optional altitude-varying atmosphere callback |

---

### `FireSolution` (`include/ballistics/fire_control.hpp`)

Output from the elevation solver for a static target.

| Field | Type | Description |
|---|---|---|
| `elevation_deg` | `double` | Launch elevation above horizontal (degrees) |
| `flight_time_ms` | `double` | Time of flight to impact (ms) |
| `valid` | `bool` | Whether a solution was found within range |

---

### `InterceptSolution` (`include/ballistics/fire_control.hpp`)

Extended output for a moving target intercept.

| Field | Type | Description |
|---|---|---|
| `fire` | `FireSolution` | Elevation and flight time |
| `azimuth_deg` | `double` | Firing bearing (0° = North, clockwise) |
| `intercept_point` | `Vec3` | Predicted target position at impact |
| `lead_distance_m` | `double` | Horizontal lead distance from current target |
| `slew_time_s` | `double` | Estimated launcher rotation time |
| `iterations` | `int` | Fixed-point convergence iterations used |
| `valid` | `bool` | Whether a valid intercept was found |

---

### `SolveParams` and `SolveResult` (`include/ballistics/async_solver.hpp`)

Input and output for the background `AsyncSolver`.

**`SolveParams`** packages all information needed for a complete fire-control solve:

| Field | Description |
|---|---|
| `launcher_pos` | World-space launcher body centre |
| `target_pos` / `target_velocity` | Current target state |
| `target_moving` | Select moving-target intercept algorithm |
| `current_azimuth_deg` / `current_elevation_deg` | Current physical orientation |
| `slew` | Launcher yaw/pitch slew rates |
| `barrel_base_offset_m` / `barrel_length_m` | Barrel geometry for muzzle position |
| `munition` | Projectile specification |
| `atmosphere` | Atmospheric conditions |

**`SolveResult`** carries everything needed by the caller after the solve completes:

| Field | Description |
|---|---|
| `valid` | Whether a solution was found |
| `azimuth_deg` / `elevation_deg` | Computed firing angles |
| `flight_time_s` | Time of flight |
| `range_m` / `max_range_m` | Range to target and maximum achievable range |
| `trajectory` | Arc sampled at 60 Hz as `std::vector<Vec3>` |
| `muzzle_pos` | Barrel tip world position |
| `intercept_point` / `lead_distance_m` | Moving-target intercept data |

---

## Trajectory Computation

### Coordinate System

The library uses a right-handed, Z-up world frame:

- **+X** = East
- **+Y** = North
- **+Z** = Up

Azimuth follows compass convention: 0° = North, 90° = East, measured clockwise.

---

### Equations of Motion

At every integration step the state derivative is computed as:

```
dpos/dt = vel

dvel/dt = gravity + drag_acceleration
        = [0, 0, -9.80665] + a_drag   (m/s²)
```

The drag acceleration uses a standard quadratic drag model:

```
v_rel      = v_projectile − v_wind
a_drag     = −k_scaled · |v_rel| · v_rel

k          = ½ · Cd · ρ₀ · A_ref / m          (precomputed, units: 1/m)
k_scaled   = k · (ρ_current / ρ_reference)     (scaled for current altitude)
```

Breaking this down physically:
- **Cd** (drag coefficient) — aerodynamic shape factor; a higher value means more drag
- **ρ** (air density) — thinner air at altitude reduces drag proportionally
- **A_ref** (reference area) — larger cross-section increases drag force
- **m** (mass) — heavier projectiles resist deceleration (Newton's second law)
- **|v_rel|** — drag force grows with the *square* of relative speed (one factor in the scalar, one in the vector)
- **v_wind** — wind shifts the reference frame; a headwind increases effective speed and therefore drag, a tailwind reduces it

When `drag_coefficient == 0`, the derivatives fast-path skips all drag computation and simulates a vacuum trajectory under gravity alone.

---

### Integration Methods

#### 4th-Order Runge-Kutta (RK4) — Default

RK4 computes four slope estimates across each timestep and combines them with a weighted average:

```
k1 = derivatives(state)
k2 = derivatives(state + dt/2 · k1)
k3 = derivatives(state + dt/2 · k2)
k4 = derivatives(state + dt   · k3)

next_state = state + (dt / 6) · (k1 + 2·k2 + 2·k3 + k4)
```

RK4 is fourth-order accurate (global error ∝ dt⁴), which makes it suitable for precision trajectory work. At `dt = 1/240 s`, each RK4 step costs approximately 200 ns on modern hardware, allowing thousands of simultaneous projectiles at 60 Hz.

#### Symplectic Euler — Fast Path

An alternative first-order integrator for throughput-only scenarios:

```
v_next = v + dt · a(pos, v)
p_next = p + dt · v_next
```

Velocity is updated before position (symplectic rather than explicit), which provides better long-term energy conservation than naive forward Euler. It is less accurate per step than RK4 but faster for benchmarking or approximate simulations.

---

### Altitude-Varying Atmosphere

When `SimulationConfig::atmosphere_fn` is provided, the simulator queries fresh atmospheric conditions at each step using the projectile's current altitude:

```cpp
double rho = cfg.atmosphere_fn
    ? cfg.atmosphere_fn(state.position.z).air_density_kg_m3
    : atmosphere_.air_density_kg_m3;
```

This integrates directly with the ISA model in `atmosphere.cpp`, which computes temperature, pressure, and density for altitudes from 0 to 20 km:

- **Troposphere (0–11 km):** Temperature decreases linearly at 6.5 K/km; pressure follows a power law.
- **Lower stratosphere (11–20 km):** Isothermal at 216.65 K; pressure decays exponentially.

---

### Ground Impact Detection

When a step crosses the ground plane (`position.z < ground_z`), the exact impact state is recovered by linear interpolation on the crossing fraction:

```
frac   = (state.z − ground_z) / (state.z − next.z)
impact = state + frac · (next − state)
```

The impact callback is then invoked and the simulation terminates.

---

## Fire-Control Solvers

### Static Target: `solve_elevation()` (`fire_control.cpp`)

Finds the launch elevation angle that causes the projectile to land at a given horizontal range from the launcher.

**Algorithm — Hybrid Ternary Search + Bisection:**

1. **Ternary search** (60 iterations) over [−87°, +87°] to locate `θ_max`, the elevation that maximises range.
2. **Bisect** the chosen interval — `[−87°, θ_max]` for a low-angle solution or `[θ_max, +87°]` for a high-angle solution — until the simulated range matches the target within the specified tolerance.

Each iteration runs a complete trajectory simulation, so a typical solve requires 120–180 simulations and takes 5–100 ms depending on range and munition.

Two detection modes handle altitude asymmetry:
- **Descending detection** (default): registers impact when the projectile is descending through the target altitude. Used when the launcher is at or above the target.
- **Ascending detection** (fallback): registers the first crossing on the way up for short-range shots to elevated targets.

---

### Moving Target: `solve_moving_target()` (`fire_control.cpp`)

Computes azimuth, elevation, and flight time to intercept a target with constant velocity.

**Algorithm — Fixed-Point on Flight Time:**

```
T₀ = initial flight time estimate

Iterate:
  1. intercept = target_pos + target_vel · Tₙ
  2. Solve elevation to reach intercept → T_{n+1}
  3. If |T_{n+1} − Tₙ| < 1 ms: converged
  4. Else: Tₙ = T_{n+1}, repeat
```

Convergence typically requires 2–5 iterations for subsonic targets. The azimuth to the intercept point is computed from the solved geometry once the loop converges.

---

### Slew-Aware Intercept: `solve_moving_target_slewed()` (`fire_control.cpp`)

Extends the moving-target solver to account for the time the physical launcher needs to rotate to the firing angles. This is important when the target moves a meaningful distance during the slew.

**Algorithm — Outer Fixed-Point on Slew Time:**

```
T_slew₀ = 0

Iterate:
  1. deferred_pos = target_pos + target_vel · T_slew_n
  2. Solve moving-target intercept from deferred_pos → azimuth, elevation
  3. T_slew_{n+1} = max(|Δazimuth| / yaw_rate, |Δelevation| / pitch_rate)
  4. If |T_slew_{n+1} − T_slew_n| < 10 ms: converged
  5. Else: repeat
```

Converges in ≤ 5 iterations for typical scenarios.

---

### Pre-computed Table: `FireControlTable` (`fire_control.hpp`)

For per-frame lookups (e.g., a game loop), pre-computing a range → (elevation, flight_time) table avoids the 5–100 ms cost of a live solve.

**Build (`build()`):**

1. Quick ternary search (20 iterations) to find `θ_max`.
2. Sweep 500 elevation angles across the search interval, simulating each at a coarser `dt = 1/120 s`.
3. Record (range, elevation, flight_time) for each angle.
4. Extract the monotone portion and sort by range.

Build takes 50–300 ms and is designed to run once, typically on a background thread.

**Lookup (`lookup()`):**

Binary search for the bounding range entries, then linear interpolation. Runs in under 1 µs — safe for every frame in a real-time application.

---

## Async Solver (`AsyncSolver`)

`AsyncSolver` wraps the fire-control solve in a non-blocking interface for use in 60 Hz update loops.

```
Main thread                         Worker thread
──────────────                      ────────────────
request(params)  ──── std::async ──► solve()
poll()           ◄─── future ─────── [running...]
result()         ◄─── swap ────────── complete
```

- `request()` queues a new solve. If a solve is already in flight, the new parameters are held and started immediately when the current solve finishes.
- `poll()` checks whether the background future is ready (non-blocking, < 1 µs). Returns `true` once when a new result becomes available.
- `result()` returns a const reference to the most recently completed `SolveResult`.

**Typical 60 Hz frame:**

```cpp
solver.request(params);         // non-blocking queue
if (solver.poll()) {
    // new result arrived — update visuals
}
const auto& r = solver.result();
if (r.valid) render(r.trajectory);
```

---

## Drag Effects on the Flyout

Drag is the dominant force shaping a trajectory beyond a few hundred metres. Its effects accumulate progressively across the flight:

**Velocity bleed-off.** The quadratic `|v_rel|²` dependence means drag is strongest at launch when the projectile is fastest, and weakens as the projectile slows. A high-velocity rifle round loses a larger fraction of its velocity in the first few hundred metres than the last.

**Trajectory flattening vs. stretching.** Compared to a vacuum trajectory, drag shortens range and increases drop at a given range. The maximum-range elevation shifts above 45° because the projectile must trade some horizontal distance for a slower descent that drag cannot cut short as severely.

**Altitude dependence.** Air density falls with altitude. A projectile fired at a steep angle travels through thinner air near its apogee, reducing drag there and causing the descending leg of the arc to be less symmetrical than the ascending leg.

**Wind coupling.** Because drag is applied in the *relative* frame (`v_projectile − v_wind`), a headwind accelerates deceleration — the projectile bleeds velocity faster — while a tailwind partially offsets drag. A crosswind introduces lateral deceleration on top of the lateral drift from the wind itself.

**Ballistic coefficient as a summary metric.** `BC = m / (Cd × A_ref)` compactly captures how a projectile resists drag: a higher BC retains speed over distance. The `.338 Lapua` (BC ≈ 0.627 kg/m²) is much more drag-resistant than the `9mm FMJ` (BC ≈ 0.255 kg/m²), which is why precision rifles use high-BC bullets.

---

## Performance Characteristics

| Operation | Typical Cost | Notes |
|---|---|---|
| `TrajectorySimulator::step()` RK4 | ~200 ns | Per step, modern hardware at -O3 |
| `solve_elevation()` | 5–100 ms | ~120–180 trajectory simulations |
| `solve_moving_target()` | 50–500 ms | Multiple `solve_elevation()` calls |
| `FireControlTable::build()` | 50–300 ms | 500 elevation sweeps at coarse dt |
| `FireControlTable::lookup()` | < 1 µs | Binary search + linear interpolation |
| `AsyncSolver::poll()` | < 1 µs | Non-blocking future check |
| 4 substeps at 240 Hz per projectile | < 1 µs | Per projectile per 16.7 ms frame |

The 200 ns per RK4 step means the simulator can advance thousands of independent projectiles every frame at 60 Hz before consuming a meaningful CPU budget.
