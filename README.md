# Ballistics-Lib

A C++17 library for real-time ballistic trajectory simulation and fire-control
computation. Designed for integration into game engines, simulation tools, and
interactive applications.

## Features

- **Trajectory simulation** — RK4 (default) or symplectic-Euler integration with
  drag, gravity, and wind. A single step costs ~200 ns; thousands of concurrent
  projectiles are feasible at 60 Hz.
- **Fire-control solver** — `solve_elevation` finds the required launch angle for a
  given range using ternary-search + bisection over full trajectory simulations.
- **Pre-computed range tables** — `FireControlTable` sweeps elevation angles once
  (~50 – 300 ms, suitable for a background thread) and serves per-frame lookups in
  under 1 µs via binary search with linear interpolation.
- **ISA atmosphere model** — International Standard Atmosphere from sea level to
  20 km; moist-air density correction via the Magnus formula.
- **Munition library** — load projectile specifications from JSON; includes five
  real-world reference rounds out of the box.
- **Interactive renderer** — a standalone 3D visualisation application (raylib +
  raygui) that draws the launcher, target, and trajectory arc in real time.

## Coordinate convention

All APIs use a consistent **right-handed, z-up** world frame:

| Axis | Direction |
|---|---|
| +x | East |
| +y | North |
| +z | Up |

Angles follow standard conventions: azimuth 0° = North, 90° = East; elevation 0° =
horizontal, 90° = straight up.

## Requirements

- CMake 3.16+
- A C++17-capable compiler (GCC, Clang, or MSVC)
- Internet access at configure time (nlohmann/json is fetched via `FetchContent`;
  raylib and raygui are fetched only when examples are enabled)

## Building

```bash
# Library only
cmake -B build
cmake --build build

# Library + examples (includes the 3D renderer)
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build

# Library + tests
cmake -B build -DBALLISTICS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Quick start

```cpp
#include <ballistics/ballistics.hpp>
using namespace ballistics;

// Load a munition
MunitionLibrary lib;
lib.load("data/munitions.json");
const MunitionSpec& m855 = lib.get("5.56x45_m855_62gr");

// Set up simulator (sea-level ISA, no wind)
AtmosphericConditions atmo = isa_conditions(0.0);
TrajectorySimulator sim(m855, atmo);

// Solve for a 600 m shot due North
LauncherOrientation orient{ .azimuth_deg = 0.0 };
FireSolution sol = solve_elevation(sim, orient, 600.0, /*muzzle_speed=*/930.0);

if (sol.valid)
    printf("Elevation %.2f deg,  ToF %.0f ms\n",
           sol.elevation_deg, sol.flight_time_ms);
```

---

## Repository layout

| Path | Contents |
|---|---|
| [`include/`](include/README.md) | Public C++17 headers — the full API surface |
| [`src/`](src/README.md) | Library implementation files |
| [`tests/`](tests/README.md) | 24 unit and performance tests |
| [`data/`](data/README.md) | `munitions.json` — five reference projectile specs |
| [`examples/`](examples/README.md) | Runnable demos and the 3D renderer |

## Performance reference

| Operation | Typical cost |
|---|---|
| `TrajectorySimulator::step()` (RK4) | ~200 ns |
| `solve_elevation()` | 5 – 100 ms (120 – 180 simulations) |
| `FireControlTable::build()` | 50 – 300 ms (async-friendly) |
| `FireControlTable::lookup()` | < 1 µs |
