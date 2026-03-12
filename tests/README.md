# tests/

Unit and performance tests for Ballistics-Lib.

## Running

```bash
cmake -B build -DBALLISTICS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Contents

### `test_trajectory.cpp`

Single executable (`test_trajectory`) covering 24 test cases across all library
modules. Uses a minimal hand-rolled test framework with no external dependencies.

**Math utilities**
- `Vec3` arithmetic, dot and cross products, norm, normalisation

**Atmosphere**
- ISA conditions at sea level, 5 km, and the tropopause (11 km)
- Moist-air density with humidity
- Stratopause clamping above 20 km

**Munitions**
- JSON loading from file and from string
- `MunitionLibrary` operations: `get`, `contains`, `names`, `size`, `clear`
- Validation errors (missing fields, invalid JSON, unknown name)
- Derived quantities: ballistic coefficient, volume

**Trajectory simulation**
- Vacuum (zero-drag) parabolic motion — compared to analytical solution
- Drag reduces range vs. vacuum
- Tailwind increases range
- Real-time `step()` performance (< 100 µs per step)
- Streaming callback interface
- Altitude-varying atmosphere callback
- Ground intersection with sub-step linear interpolation

**Fire control**
- Low-angle and high-angle `solve_elevation` solutions
- Out-of-range detection
- Elevated launcher
- Azimuth independence in still air
- Wind effect on required azimuth
- `FireControlTable`: build, ready state, lookup accuracy vs. `solve_elevation`
  (< 0.2° error with 500 samples), out-of-range handling, real-time performance
  (< 1 µs per lookup), high-angle table, below-minimum-range handling
