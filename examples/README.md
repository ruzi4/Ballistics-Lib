# examples/

Runnable programs that demonstrate library usage patterns.

Build all examples:

```bash
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build
```

Each binary is placed in `build/examples/` with a `data/` subdirectory copied
alongside it automatically.

---

## `realtime_demo`  —  `realtime_demo.cpp`

Demonstrates integrating `TrajectorySimulator` into a 60 Hz game loop.

- Loads the `5.56x45_m855_62gr` munition from `data/munitions.json`
- Fires at 45° elevation, 930 m/s muzzle velocity into a 5 m/s headwind
- Runs the projectile at **60 Hz** with **4 physics sub-steps per frame** (240 Hz integration)
- Measures per-frame wall time and reports impact position, range, and speed
- Cross-validates against a batch offline simulation for accuracy

Intended as a starting point for embedding trajectory simulation in a
real-time application or game engine.

---

## `fire_control_async`  —  `fire_control_async.cpp`

Demonstrates the `FireControlTable` async pattern for per-frame fire solutions
without stalling the render thread.

- Implements a `FireControlSystem` wrapper that builds the lookup table on a
  background thread (`std::async`) and hot-swaps it when ready
- Simulates 120 frames (2 seconds) at 60 Hz with a moving target (100 – 1200 m)
- Triggers a weapon and atmosphere change at frame 60 to show mid-session rebuild
- Reports per-frame lookup latency (< 1 µs) and async build completion time

---

## `ballistics_renderer`  —  `renderer.cpp`

Interactive 3D visualisation of the full fire-solution pipeline.

Requires **raylib 5.0** and **raygui 4.0**, both fetched automatically via
CMake `FetchContent`.

For full documentation of the renderer's capabilities and controls see
**[RENDERER_README.md](RENDERER_README.md)**.

**Quick summary:**
- 3D scene: box-and-barrel launcher (rotates to computed azimuth/elevation),
  red muzzle direction ray, target sphere, orange trajectory arc, apex and
  impact markers
- Left-hand GUI panel: munition dropdown, muzzle speed slider, position sliders
  for launcher and target, live fire-solution readout
- Right-mouse drag to orbit; scroll wheel to zoom
- Keyboard shortcuts to move the launcher and target (see linked README)
