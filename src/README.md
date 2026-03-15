# src/

Implementation files for the `ballistics` static library.

| File | Responsibility |
|---|---|
| `atmosphere.cpp` | ISA atmosphere model and moist-air density computation |
| `munition.cpp` | JSON loading and validation for `MunitionSpec` / `MunitionLibrary` |
| `trajectory.cpp` | RK4 and symplectic-Euler integrators, batch simulation, ground intersection |
| `fire_control.cpp` | `solve_elevation`, `solve_moving_target`, `solve_moving_target_slewed`, and `FireControlTable` |
| `async_solver.cpp` | `solve()` synchronous solver and `AsyncSolver` non-blocking wrapper for game loops |

All five files are compiled into a single static library target (`ballistics`) by
the root `CMakeLists.txt`. They have no public symbols beyond what is declared in
the corresponding headers under `include/ballistics/`.

## Physics summary

- **Gravity**: −9.80665 m/s² in the z-up direction
- **Drag**: `F_d = −½ · Cd · ρ · A · |v_rel|² · v̂_rel`, where `v_rel = v − wind`
- **Integration**: 4th-order Runge-Kutta by default; symplectic Euler available
- **Coordinate convention**: x = East, y = North, z = Up (right-handed, z-up)
- **Air density**: precomputed at construction as `k = ½ · Cd · ρ · A / m`;
  scaled per-step when an altitude-varying `atmosphere_fn` callback is provided

## Implementation notes

### `atmosphere.cpp`
Implements the ISA troposphere (0–11 km) with a linear temperature lapse of
6.5 K/km and the lower stratosphere (11–20 km) isothermal at 216.65 K.
Above 20 km the implementation clamps to stratopause values rather than crashing.
Moist-air density uses the Magnus formula for water vapour saturation pressure.

### `trajectory.cpp`
The core RK4 integrator (`step_rk4_rho`) evaluates `derivatives()` four times
per step (k1–k4) then combines them as `(k1 + 2k2 + 2k3 + k4) / 6`. The
altitude-varying density path passes the caller-supplied `rho` to each
`derivatives()` call without re-reading the stored atmosphere, saving two
multiplications per step.

Ground intersection is detected when the projectile descends through
`cfg.ground_z`. The exact crossing time is found by linear interpolation between
the last above-ground and first below-ground states. For launchers below the
target plane, ascending crossings are detected via the streaming callback.

### `fire_control.cpp`
`solve_elevation` uses a ternary search to bracket the elevation that maximises
range, then bisects to find the exact angle that hits the requested range, running
a full trajectory simulation at each iteration (~120–180 total).

`solve_moving_target` wraps `solve_elevation` in an iterative fixed-point loop:
it estimates the flight time T, forward-projects the target by T, re-solves
elevation to the projected position, and repeats until convergence (|ΔT| < 1 ms).
Typical convergence: 2–5 iterations.

`solve_moving_target_slewed` adds an outer fixed-point loop that accounts for the
launcher's slew time. At each outer iteration it estimates how long the launcher
takes to rotate to the required firing angles, defers the target forward-projection
by that time, re-invokes `solve_moving_target`, and iterates until |ΔT_s| < 10 ms.

`FireControlTable::build` sweeps `num_samples` elevation angles at a coarsened
timestep (1/120 s) for speed, simulates each, then extracts the monotone
range-elevation curve. `lookup` performs binary search with linear interpolation
in O(log N) time.

### `async_solver.cpp`
`solve()` encapsulates the full fire-control pipeline: it computes range and
azimuth from launcher to target, builds a quick diagnostic table for max-range
reporting, dispatches to `solve_elevation` (static target) or
`solve_moving_target_slewed` (moving target), and generates the trajectory arc
at 60 Hz for visualisation.

`AsyncSolver` wraps `solve()` in a `std::async` background thread with a
request/poll/result interface suitable for 60 Hz game loops. Requests that arrive
while a solve is in flight are queued and started automatically when the current
solve completes (checked during `poll()`).

See [`include/README.md`](../include/README.md) for the full public API.
