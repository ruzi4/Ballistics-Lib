# examples/

Runnable programs that demonstrate library usage patterns.

Build all examples:

```bash
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build
```

On Windows with MSVC, add `--config Release` (or `Debug`) to the build step:

```bat
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build --config Release
```

Each binary is placed in `build/examples/` on single-config generators, or
`build/examples/Release/` (or `Debug/`) with MSVC multi-config generators.
A `data/` subdirectory containing `munitions.json` and `launcher_config.json`
is copied alongside each binary automatically at build time.

---

## `realtime_demo`  —  `realtime_demo.cpp`

Demonstrates integrating `TrajectorySimulator` into a 60 Hz game loop.

- Loads the `5.56x45_m855_62gr` munition (4.02 g, Cd=0.307, 930 m/s) from
  `data/munitions.json`
- Fires at **45° elevation**, 930 m/s muzzle velocity into a **5 m/s headwind**
  (ISA sea-level atmosphere)
- Runs at **60 Hz** with **4 RK4 sub-steps per frame** (240 Hz physics integration)
- Uses ground-crossing linear interpolation for accurate impact position and time
- Measures per-frame wall time; warns if any frame exceeds the 16.7 ms budget
- Cross-validates the real-time result against a batch `simulate()` offline run

Intended as a starting point for embedding trajectory simulation in a
real-time application or game engine.

---

## `fire_control_async`  —  `fire_control_async.cpp`

Demonstrates the `FireControlTable` async pattern for per-frame fire solutions
without stalling the render thread.

- Implements a `FireControlSystem` wrapper that builds the lookup table on a
  background thread via `std::async` and hot-swaps it atomically when ready
- Simulates **120 frames (2 s) at 60 Hz** with a moving target sweeping
  100–1200 m range
- Triggers a **weapon and atmosphere change at frame 60** to demonstrate
  mid-session table rebuild without blocking the main loop
- Reports per-frame lookup latency (< 1 µs) and async build completion time
- Links against `Threads::Threads` for portable `std::async` on all platforms

---

## `ballistics_renderer`  —  `renderer.cpp`

Interactive 3D visualisation of the full fire-solution pipeline, including
slew-aware moving-target intercept.

Requires **raylib 5.0** and **raygui 4.0**, both fetched automatically via
CMake `FetchContent`. No manual dependency installation required.

For full documentation of controls and the GUI see
**[RENDERER_README.md](RENDERER_README.md)**.

**Quick summary:**
- 3D scene: box-and-barrel launcher (rotates to the computed azimuth and
  elevation), red muzzle direction ray, target sphere, orange trajectory arc,
  yellow apex marker, gold impact marker, white 10-m ground grid, R/G/B axis
  arrows
- **Moving-target mode**: enable a ping-pong target with configurable speed and
  heading; the solver calls `solve_moving_target_slewed` (accounting for
  launcher slew time from `launcher_config.json`) and shows lead distance
- Left-hand GUI panel: munition dropdown, muzzle speed slider, launcher and
  target position sliders, moving-target controls, live fire-solution readout
- Background solver thread keeps the renderer responsive during the 5–100 ms
  solve time
- Right-mouse drag to orbit; scroll wheel to zoom; middle-mouse drag to pan
- **View Focus** dropdown to snap the orbit centre to the launcher, target, or
  a freely-panned point
- Keyboard shortcuts to move launcher and target (see `RENDERER_README.md`)
- On Windows, opens only the graphics window (no redundant console window)
