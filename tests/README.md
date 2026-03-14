# tests/

Unit and performance tests for Ballistics-Lib.

## Running

```bash
cmake -B build -DBALLISTICS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

On Windows (MSVC), add `--build-config Release` to the ctest invocation.

## Contents

### `test_trajectory.cpp`

Single executable (`test_trajectory`) covering 40 test functions across all library
modules. Uses a minimal hand-rolled test framework with no external dependencies.

---

### Math utilities

- `Vec3` arithmetic (addition, subtraction, scalar multiply/divide)
- Dot product, cross product, norm, normalisation

---

### Atmosphere

- ISA conditions at sea level, 5 km, and the tropopause (11 km)
- Moist-air density with humidity correction (Magnus formula)
- Stratopause clamping: values above 20 km are clamped rather than producing NaN

---

### Munitions

- JSON loading from file (`MunitionLibrary::load`) and from string
  (`load_from_string`)
- Library operations: `get`, `contains`, `names`, `size`, `clear`
- Validation: missing required fields throw `std::invalid_argument`; bad JSON
  throws `std::runtime_error`; unknown name throws `std::out_of_range`
- Derived quantities: `ballistic_coefficient()`, `volume_m3()`

---

### Trajectory simulation

- Vacuum (zero-drag, `Cd = 0`) parabolic motion — compared to analytical solution
- Drag reduces range vs. vacuum case
- Tailwind increases range; headwind decreases range
- Real-time `step()` performance (< 100 µs per step on any reasonable machine)
- Streaming callback interface (`simulate` with `StepCallback`)
- Altitude-varying atmosphere via `SimulationConfig::atmosphere_fn` callback
- Ground intersection with sub-step linear interpolation for accurate impact time

---

### Fire control — `solve_elevation`

- Low-angle and high-angle solutions to the same range
- Out-of-range detection (`valid = false`)
- Elevated launcher: `launch_height_m` above/below the target plane
- Azimuth independence in still air (elevation identical for N/E/S/W)
- Wind effect: heading relative to wind changes required elevation
- Non-zero `target_altitude_m` (separate target plane from ground)
- Invalid muzzle speed guard (`muzzle_speed_ms ≤ 0` throws)

---

### Fire control — `FireControlTable`

- Build without error, `ready()` transitions from false to true
- Lookup accuracy: agrees with `solve_elevation` within 0.2° (500-sample table)
- Out-of-range and negative-range return `valid = false`
- Below-minimum-range handling (minimum entry is valid; half that entry is not)
- Real-time performance: < 1 µs per lookup (10 000 consecutive lookups timed)
- Build timing: reported for informational purposes; < 5 s on any machine
- High-angle table: build and lookup with `high_angle = true`
- Wind + azimuth: table built with a non-zero wind direction
- `num_samples` guard: values < 2 are clamped to 2
- Target altitude: table built with non-zero `target_altitude_m`

---

### Altitude variation

Tests cover launcher and target at different altitudes, including cases where the
launcher is significantly above the target (shooting down) and below the target
(ascending-path hit). All scenarios are cross-validated by re-simulating the
returned solution and checking impact position.

- Launcher above target (multiple munition/altitude pairs)
- Launcher below target — short-range ascending-path solutions
- Both launcher and target at non-zero altitudes
- `FireControlTable` built with non-zero `launch_height_m` and `target_altitude_m`
- Flight times remain within the 60 s simulation ceiling

---

### Moving-target intercept

- Stationary target: `solve_moving_target` agrees with `solve_elevation`
- Level moving target: lead distance is positive and proportional to target speed
- Elevated target above launcher: ascending-path intercept, `elevation_deg > 0`
- Launcher above target: intercept shoots downward, `elevation_deg < 0`
- `solve_moving_target_slewed` basic: `slew_time_s ≈ 0` when already on-target;
  `slew_time_s > 2 s` when 90° off at 25°/s slew rate
- Slew + elevated target: `elevation_deg > 0`, `slew_time_s > 2 s`,
  `lead_distance_m > 0`, intercept altitude near target plane
- Out-of-range: returns `valid = false` when target is beyond maximum range
