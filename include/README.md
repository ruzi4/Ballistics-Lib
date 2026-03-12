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

### `ballistics/trajectory.hpp`
Core simulation API.

- **`ProjectileState`** — position (`Vec3`, m), velocity (`Vec3`, m/s), time (s)
- **`SimulationConfig`** — timestep, max time, ground altitude, integrator choice,
  optional altitude-varying atmosphere callback
- **`TrajectorySimulator`** — constructed from a `MunitionSpec` and
  `AtmosphericConditions`; exposes:
  - `step()` — single integration step for 60 Hz game loops (~200 ns/call)
  - `simulate()` — batch run to ground impact; returns all states or streams via callback

### `ballistics/fire_control.hpp`
Targeting and pre-computed range tables.

- **`LauncherOrientation`** — azimuth in degrees (0 = North, 90 = East)
- **`FireSolution`** — elevation angle (°), time of flight (ms), validity flag
- **`solve_elevation()`** — ternary-search + bisection solver; accurate but slow
  (5 – 100 ms); suitable for offline / async use
- **`FireControlTable`** — pre-computed range → (elevation, flight-time) table;
  `build()` runs once in ~50 – 300 ms; `lookup()` is O(log N), < 1 µs per call

### `ballistics/munition.hpp`
Projectile specification.

- **`MunitionSpec`** — mass, density, reference area, drag coefficient, diameter;
  derived `ballistic_coefficient()` and `volume_m3()`
- **`MunitionLibrary`** — loads one or more munitions from JSON files or strings;
  retrieve by name with `get()`

### `ballistics/atmosphere.hpp`
Atmospheric conditions and models.

- **`Wind`** — 3D wind velocity vector (m/s, world space)
- **`AtmosphericConditions`** — temperature, pressure, humidity, air density, wind
- **`compute_air_density()`** — ideal gas law with Magnus-formula humidity correction
- **`isa_conditions()`** — International Standard Atmosphere from 0 – 20 km

### `ballistics/math/vec3.hpp`
Double-precision 3D vector (`Vec3`) with fully inlined arithmetic, dot/cross
products, and norms.

### `ballistics/math/math_constants.hpp`
`kDegToRad` and `kRadToDeg` conversion constants.
