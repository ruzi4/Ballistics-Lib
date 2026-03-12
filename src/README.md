# src/

Implementation files for the `ballistics` static library.

| File | Responsibility |
|---|---|
| `atmosphere.cpp` | ISA atmosphere model and moist-air density computation |
| `munition.cpp` | JSON loading and validation for `MunitionSpec` / `MunitionLibrary` |
| `trajectory.cpp` | RK4 and symplectic-Euler integrators, batch simulation, ground intersection |
| `fire_control.cpp` | `solve_elevation` bisection solver and `FireControlTable` pre-computed lookup table |

All four files are compiled into a single static library target (`ballistics`) by
the root `CMakeLists.txt`. They have no public symbols beyond what is declared in
the corresponding headers under `include/ballistics/`.

## Physics summary

- **Gravity**: −9.80665 m/s² in the z-up direction
- **Drag**: `F_d = −½ · Cd · ρ · A · |v_rel|² · v̂_rel`, where `v_rel = v − wind`
- **Integration**: 4th-order Runge-Kutta by default; symplectic Euler available
- **Coordinate convention**: x = East, y = North, z = Up (right-handed, z-up)
- **Air density**: computed once per `TrajectorySimulator` construction and cached
  as the drag constant `k = ½ · Cd · ρ · A / m`

See [`include/README.md`](../include/README.md) for the full public API.
