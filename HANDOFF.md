# Ballistics-Lib Renderer: Session Handoff

## Branch
`claude/add-windows-portability-mMUzN`

## Repository Overview
C++17 ballistic trajectory simulation library with an interactive 3D renderer (`examples/renderer.cpp`) built on raylib 5.0 + raygui 4.0. The renderer places a launcher and target in a scene, computes a fire solution asynchronously via `solve_elevation()`, and draws the trajectory arc.

### Coordinate Systems
- **Ballistics lib**: x = East, y = North, z = Up (right-handed, z-up)
- **raylib**: x = East, y = Up, z = South (right-handed, y-up)
- **Transform**: `rl = { ball.x, ball.z, -ball.y }` (see `to_rl()` at `examples/renderer.cpp:37`)

### Key Files
| File | Purpose |
|------|---------|
| `examples/renderer.cpp` | Single-file renderer (raylib/raygui), all UI + 3D rendering |
| `src/fire_control.cpp` | `solve_elevation()` — ternary search + bisection fire solver |
| `include/ballistics/fire_control.hpp` | `FireSolution`, `LauncherOrientation`, `FireControlTable` |
| `src/trajectory.cpp` | `TrajectorySimulator::simulate()` — RK4/Euler integration |
| `include/ballistics/trajectory.hpp` | `ProjectileState`, `SimulationConfig` |
| `data/munitions.json` | 5 munition specs (9mm, 5.56, 7.62, .338 Lapua, .50 BMG) |
| `CMakeLists.txt` | Root build (cmake_minimum_required set to 3.20) |
| `examples/CMakeLists.txt` | Fetches raylib 5.0 + raygui 4.0 via FetchContent |

---

## Changes Already Completed (This Session)

### 1. Extended View Distance
- `examples/renderer.cpp:434` — Grid increased from `DrawGrid(300, 10.0f)` to `DrawGrid(800, 10.0f)` (8km x 8km)
- `examples/renderer.cpp:381` — Zoom is now proportional to distance: `cam_dist * 0.08f` instead of fixed `10.f`

### 2. Added Pan Controls (Middle-Mouse Drag)
- State vars at `renderer.cpp:262-263`: `float pan_x = 0.f, pan_z = 0.f`
- Pan input at `renderer.cpp:372-379`: middle-mouse drag computes delta in camera's local horizontal plane, scaled by `cam_dist * 0.002f`
- Pan offset applied to focus point at `renderer.cpp:397-398`
- Pan only works in Free mode (`cam_focus_mode == 0`)

### 3. Added View Focus Dropdown (Snap-to-Object)
- State at `renderer.cpp:265-268`: `int cam_focus_mode` (0=Free, 1=Launcher, 2=Target)
- Focus logic at `renderer.cpp:394-398`: mode selects orbit center (midpoint, launcher pos, or target pos)
- Pan resets when switching away from Free mode (`renderer.cpp:385-388`)
- Dropdown drawn deferred at `renderer.cpp:567-568`

### 4. Fixed Dropdown Z-Order
- Both dropdowns (munition + view focus) are drawn **last** in the panel, after all sliders and labels
- Munition dropdown position saved at `renderer.cpp:481`, drawn at `renderer.cpp:571-572`
- View focus dropdown position saved at `renderer.cpp:550`, drawn at `renderer.cpp:567-568`
- This ensures expanded dropdown lists render on top of slider controls

### 5. Updated Controls Help
- Added "Middle-mouse drag — pan camera" at `renderer.cpp:563`
- Expanded help box height from 178/180 to 194/198 pixels

### 6. Windows Portability (this session)
- `CMakeLists.txt`: added `if(MSVC) add_compile_options(/utf-8) endif()` so `µ`
  in `printf` strings renders correctly regardless of Windows ANSI code page
- `CMakeLists.txt`: added `-Wno-nan-infinity-disabled` for Clang to silence a
  third-party nlohmann_json warning triggered by `-ffast-math`
- `tests/CMakeLists.txt`: added `POST_BUILD` copy of `data/` using
  `$<TARGET_FILE_DIR:test_trajectory>` so CTest finds `munitions.json` in MSVC
  multi-config subdirectories (`Release/`, `Debug/`, etc.)
- `examples/CMakeLists.txt`: added `WIN32` keyword to `ballistics_renderer` so
  Windows shows only the graphics window (no redundant console window)
- `.github/workflows/ci.yml` (new): matrix CI covering Windows/MSVC,
  Linux/GCC, and Linux/Clang; examples excluded from headless CI
- `src/fire_control.cpp`: copied structured binding `theta_max` to a plain
  `const double` before the lambda — Clang flagged the direct capture as a
  C++20 extension

---

## Open Issue: Fire Solution Fails at Certain Altitudes

### Problem
When the target or launcher is at certain altitudes, the fire solution returns `valid=false` and the UI shows "No solution (target out of range)" with no further explanation.

### Root Cause Analysis
The issue is in `solve_async()` at `renderer.cpp:76-132` and how it calls `solve_elevation()` in `src/fire_control.cpp:93-152`.

**How `solve_async()` computes parameters (renderer.cpp:79-98):**
```cpp
const double range_m = std::sqrt(dx * dx + dy * dy);  // HORIZONTAL range only (x,y)
const double launch_height = p.launcher_pos.z - p.target_pos.z;  // relative height
const double target_alt = p.target_pos.z;  // absolute altitude

FireSolution sol = solve_elevation(sim, orient, range_m,
                                   p.muzzle_speed,
                                   launch_height, /*high_angle=*/false,
                                   /*tolerance_m=*/0.5,
                                   target_alt);
```

**How `solve_elevation()` works (fire_control.cpp:93-152):**
1. Calls `find_max_range_angle()` to find the elevation angle giving maximum horizontal range
2. If `range_m > r_max`, returns `valid=false` immediately (line 118)
3. Otherwise bisects to find the angle matching the requested range

**The `shoot()` helper (fire_control.cpp:22-57):**
- Starts projectile at `z = launch_height_m` (relative height above target)
- Sets `cfg.ground_z = target_z_m` (absolute target altitude)
- Simulation stops when projectile reaches `ground_z`

**Potential failure scenarios:**
1. **High target altitude with low launcher**: `launch_height` becomes very negative (launcher below target). The `shoot()` function starts the projectile at `z = launch_height_m` (a negative value) with `ground_z = target_alt` (a positive value). The projectile starts *below* the ground plane and may never cross it, producing nonsensical range values.
2. **Both at high altitude**: The solver uses `target_altitude_m` as `ground_z`. The internal simulation in `shoot()` starts at relative position `z = launch_height_m` but the ground is at absolute `target_z_m`. When `launch_height_m` is negative (launcher below target), the projectile starts below ground_z and the trajectory terminates immediately or produces a tiny range.
3. **The "No solution" message is generic**: The renderer only shows "No solution (target out of range)" regardless of whether the issue is actual range limitation, altitude configuration, or a solver edge case.

**Key code path in `shoot()` (fire_control.cpp:36-56):**
```cpp
st.position = Vec3{0.0, 0.0, launch_height_m};  // starts at relative height
cfg.ground_z = target_z_m;                        // ground at absolute altitude
```
When `launch_height_m` is `-50` and `target_z_m` is `50`, the projectile starts at z=-50 with ground at z=50 — it's 100m below the termination plane. The simulation likely terminates immediately (projectile already below ground) or gives a degenerate range.

### What Needs to Be Done
Two things need fixing:

1. **Fix the solver invocation in `solve_async()`** — The `launch_height_m` and `target_altitude_m` parameters passed to `solve_elevation()` may be inconsistent. The `shoot()` function expects `launch_height_m` as height *above the target plane* and `ground_z` as the *absolute* altitude where the ground plane sits. When the launcher is lower than the target, `launch_height` goes negative but `target_alt` stays at the target's absolute Z — the projectile starts below the ground plane. The fix likely involves ensuring the starting position and ground plane are consistent (e.g., always use relative heights, or adjust the starting z so it's always above ground_z).

2. **Improve the failure message in the renderer** — Instead of the generic "No solution (target out of range)", show more specific diagnostic info: the horizontal range vs max range, or note when altitude difference is the likely cause.

### Relevant Code Locations for the Fix
- `examples/renderer.cpp:76-132` — `solve_async()`, especially lines 88-98 where parameters are computed
- `src/fire_control.cpp:22-57` — `shoot()` internal helper, especially lines 37 and 45
- `src/fire_control.cpp:93-152` — `solve_elevation()`, especially line 118 where it returns invalid
- `examples/renderer.cpp:544-545` — The "No solution" display message

### Build Notes
- `CMakeLists.txt` uses `cmake_minimum_required(VERSION 3.20)`
- Linux: requires X11 dev packages: `libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev`
- Linux build: `cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON && cmake --build build`
- Windows build: `cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON && cmake --build build --config Release`
- Syntax-only check (no X11 needed): `g++ -std=c++17 -fsyntax-only -I include -I build/_deps/raylib-src/src -I build/_deps/raylib-src/examples/shapes examples/renderer.cpp`
