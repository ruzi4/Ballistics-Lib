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
  Supports non-zero launch height and target altitude for terrain-aware solutions.
- **Moving-target intercept** — `solve_moving_target` computes the azimuth and
  elevation to intercept a moving target using iterative flight-time lead
  prediction. `solve_moving_target_slewed` extends this by accounting for the
  time a physical launcher takes to slew to the firing angle.
- **Pre-computed range tables** — `FireControlTable` sweeps elevation angles once
  (~50 – 300 ms, suitable for a background thread) and serves per-frame lookups in
  under 1 µs via binary search with linear interpolation.
- **Async solver** — `AsyncSolver` wraps the fire-control solvers in a non-blocking
  interface designed for 60 Hz game loops. Call `request()` when parameters change,
  `poll()` every frame, and read `result()` — the heavy computation runs on a
  background thread. Supports both static and moving-target engagements with
  automatic trajectory arc generation for visualisation.
- **ISA atmosphere model** — International Standard Atmosphere from sea level to
  20 km; moist-air density correction via the Magnus formula. Altitude-varying
  density can be applied per-step via a callback.
- **Munition library** — load projectile specifications from JSON; includes five
  real-world reference rounds out of the box.
- **Interactive renderer** — a standalone 3D visualisation application (raylib +
  raygui) that draws the launcher, target, and trajectory arc in real time,
  including slew-aware moving-target intercept.

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

- CMake 3.20+ (including CMake 4.x)
- A C++17-capable compiler (GCC 9+, Clang 10+, or MSVC 2019+)
- Internet access at configure time (nlohmann/json is fetched via `FetchContent`;
  raylib and raygui are fetched only when examples are enabled)

## Building

Both tests and examples are built by default. Pass `-DBALLISTICS_BUILD_TESTS=OFF`
or `-DBALLISTICS_BUILD_EXAMPLES=OFF` to skip either.

```bash
# Library + tests + examples (default)
cmake -B build
cmake --build build

# Library only
cmake -B build -DBALLISTICS_BUILD_TESTS=OFF -DBALLISTICS_BUILD_EXAMPLES=OFF
cmake --build build

# Run tests
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

### Windows (MSVC)

The build is fully supported on Windows with Visual Studio 2019 or later.
Use the `--config` flag to select the build configuration:

```bat
cmake -B build
cmake --build build --config Release
ctest --test-dir build --build-config Release --output-on-failure
```

Source files containing `µ` (U+00B5) are compiled with `/utf-8` automatically
so console output is correct regardless of the system ANSI code page.

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

### Game-loop integration (AsyncSolver)

```cpp
#include <ballistics/ballistics.hpp>
using namespace ballistics;

// Set up parameters once
SolveParams params;
params.launcher_pos    = { 0.0, 0.0, 0.0 };
params.target_pos      = { 0.0, 600.0, 0.0 };
params.munition        = lib.get("5.56x45_m855_62gr");
params.atmosphere      = isa_conditions(0.0);
params.muzzle_speed_ms = 930.0;

AsyncSolver solver;

// In your update loop (60 Hz):
solver.request(params);     // non-blocking — launches background thread
if (solver.poll()) {        // true when a new result was installed
    // Update cached visuals only when result changes
}

const SolveResult& r = solver.result();
if (r.valid) {
    aim_launcher(r.azimuth_deg, r.elevation_deg);
    draw_arc(r.trajectory);  // std::vector<Vec3> in world coords
}
```

---

## Repository layout

| Path | Contents |
|---|---|
| [`include/`](include/README.md) | Public C++17 headers — the full API surface |
| [`src/`](src/README.md) | Library implementation files |
| [`tests/`](tests/README.md) | Comprehensive unit and performance tests |
| [`data/`](data/README.md) | `munitions.json` — five reference projectile specs |
| [`examples/`](examples/README.md) | Runnable demos and the 3D renderer |
| [`rag/`](rag/RAG_README.md) | RAG system for querying the codebase with Claude |

## Performance reference

| Operation | Typical cost |
|---|---|
| `TrajectorySimulator::step()` (RK4) | ~200 ns |
| `solve_elevation()` | 5 – 100 ms (120 – 180 simulations) |
| `solve_moving_target()` | (max_iterations + 1) × `solve_elevation()` |
| `FireControlTable::build()` | 50 – 300 ms (async-friendly) |
| `FireControlTable::lookup()` | < 1 µs (O(log N) binary search) |
| `solve()` (static target) | 5 – 100 ms (use via `AsyncSolver`) |
| `solve()` (moving target) | 50 – 500 ms (use via `AsyncSolver`) |
| `AsyncSolver::poll()` | < 1 µs per frame |

---

## RAG — ask questions about this codebase

The `rag/` directory contains a Retrieval-Augmented Generation system that lets
any Claude interface answer questions about this library by searching the actual
source code (ChromaDB + all-MiniLM-L6-v2 embeddings, answered by `claude-opus-4-6`).

See **[rag/RAG_README.md](rag/RAG_README.md)** for full setup and usage
instructions, including the terminal CLI, Claude Code MCP integration, Cline,
and the claude.ai web app.
