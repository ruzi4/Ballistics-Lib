# Ballistics Trajectory Renderer

An interactive 3D application that lets you place a launcher and a target anywhere
in a scene, then computes and visualises the ballistic fire solution in real time.

## Building

Enable examples when configuring CMake. raylib 5.0 and raygui 4.0 are fetched
automatically at configure time — no manual dependency installation required.

```bash
# Linux / macOS
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/ballistics_renderer
```

```bat
:: Windows (MSVC)
cmake -B build -DBALLISTICS_BUILD_EXAMPLES=ON
cmake --build build --config Release
build\examples\Release\ballistics_renderer.exe
```

On Windows, `ballistics_renderer` opens only the graphics window — no
redundant console window appears.

The binary expects a `data/` directory (containing `munitions.json`) alongside it.
The CMake build copies this automatically.

---

## Scene Overview

| Object | Appearance | Description |
|---|---|---|
| Launcher | Green box + grey barrel | Rotates to the computed azimuth and elevation |
| Muzzle ray | Red line | Extends from the barrel tip in the exact firing direction |
| Target | Red sphere | Sits at the user-specified position; stake and crosshair show its ground projection |
| Trajectory | Orange arc | Full ballistic path from launcher to target |
| Apex marker | Yellow sphere | Highest point on the trajectory |
| Impact marker | Gold sphere | Where the projectile reaches the target altitude |
| Ground grid | White lines | 10 m cells; covers ±4000 m in each direction |
| Axis arrows | R/G/B lines | Red = East (+X), Green = North (−Z in scene), Blue = Up (+Y) |

---

## Coordinate System

The ballistics library uses **x = East, y = North, z = Up** (right-handed, z-up).
All position sliders in the GUI use these same units (metres).

---

## GUI Panel

The left-hand panel (340 px) contains all interactive controls.

### Munition

A dropdown lists every munition loaded from `data/munitions.json`:

| # | Name | Default muzzle speed |
|---|---|---|
| 0 | 9mm\_fmj\_115gr | 370 m/s |
| 1 | 5.56×45 M855 62gr | 930 m/s |
| 2 | 7.62×51 M80 147gr | 840 m/s |
| 3 | .338 Lapua 250gr | 915 m/s |
| 4 | .50 BMG 660gr | 930 m/s |

Switching munition automatically resets the muzzle speed to the default for
that round.

### Muzzle Speed

Slider range: **100 – 1500 m/s**. Adjust after selecting a munition to explore
how velocity affects range and trajectory shape.

### Launcher Position

| Slider | Axis | Range |
|---|---|---|
| X(E) | East / West | −2000 to +2000 m |
| Y(N) | North / South | −2000 to +2000 m |
| Alt | Altitude | 0 to 200 m |

### Target Position

| Slider | Axis | Range |
|---|---|---|
| X(E) | East / West | −2000 to +2000 m |
| Y(N) | North / South | −2000 to +2000 m |
| Alt | Altitude | 0 to 500 m |

### Fire Solution (read-only)

Updated automatically whenever any input changes. The solver runs on a
background thread so the renderer stays responsive during computation.

| Field | Description |
|---|---|
| Azimuth | Horizontal bearing from launcher to target (0° = North, 90° = East) |
| Elevation | Required launch angle above horizontal |
| Range | Horizontal ground distance from launcher to target |
| Flight T | Time of flight from launch to impact |

Possible status messages:

- **Green values** — valid solution found, trajectory is drawn
- **Computing…** — background solver is running (typically 5 – 100 ms)
- **No solution (out of range)** — target is beyond the munition's maximum range

---

## Controls

### Keyboard

| Key | Action |
|---|---|
| **W** | Move target North |
| **S** | Move target South |
| **D** | Move target East |
| **A** | Move target West |
| **E** | Raise target altitude |
| **Q** | Lower target altitude |
| **↑** | Move launcher North |
| **↓** | Move launcher South |
| **→** | Move launcher East |
| **←** | Move launcher West |
| **Page Up** | Raise launcher altitude |
| **Page Down** | Lower launcher altitude |

Movement speed is **80 m/s** (hold the key and release to stop).
Keyboard input is suspended while the munition dropdown is open.

### Mouse

| Action | Effect |
|---|---|
| Right-mouse drag | Orbit camera around the current focus point |
| Middle-mouse drag | Pan the focus point in the camera's horizontal plane |
| Scroll wheel | Zoom in / out (5 m to 8 km); zoom speed scales with distance |

### View Focus

A **View Focus** dropdown in the GUI panel controls what the camera orbits and
pans around:

| Mode | Orbit / pan centre |
|---|---|
| Free | Freely panned point (middle-mouse drag) |
| Launcher | Launcher position (pan disabled) |
| Target | Target position (pan disabled) |

Switching from Free to Launcher or Target resets the pan offset.

---

## Physics Notes

- The solver calls `solve_elevation` from the ballistics library, which uses
  a ternary-search + bisection algorithm and runs 120–180 full trajectory
  simulations per call.
- Drag, gravity, and ISA sea-level atmosphere are all applied; wind is zero
  by default.
- The trajectory arc is collected at 60 Hz integration (RK4) and converted to
  the scene's coordinate space for rendering.
- Elevated targets (target altitude > launcher altitude) are fully supported.
