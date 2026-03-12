#pragma once

#include "math/vec3.hpp"

namespace ballistics {

/// Wind vector in world-space coordinates (m/s).
/// Coordinate convention: x = East, y = North, z = Up.
struct Wind {
    Vec3 velocity_ms;  ///< World-space wind velocity (m/s)
};

/// All atmospheric quantities needed for drag computation at a given altitude.
struct AtmosphericConditions {
    double temperature_K{288.15};    ///< Air temperature (K)
    double pressure_Pa{101325.0};    ///< Absolute pressure (Pa)
    double relative_humidity{0.0};   ///< Fractional humidity [0, 1]
    double air_density_kg_m3{1.225}; ///< Pre-computed air density (kg/m³)
    Wind   wind{};                   ///< Ambient wind

    /// Re-compute air_density_kg_m3 from temperature, pressure and humidity.
    /// Call this after manually setting temperature / pressure.
    void recompute_density() noexcept;
};

/// Compute air density (kg/m³) using the ideal gas law with a humidity
/// correction via the Magnus formula for water vapour pressure.
///
/// @param temperature_K   Air temperature (Kelvin)
/// @param pressure_Pa     Total air pressure (Pascal)
/// @param relative_humidity  Fractional humidity [0, 1]  (default 0)
double compute_air_density(double temperature_K,
                           double pressure_Pa,
                           double relative_humidity = 0.0) noexcept;

/// Build an AtmosphericConditions object from the International Standard
/// Atmosphere (ISA) model at the given altitude above mean sea level.
///
/// Covers the troposphere (0–11 km) and lower stratosphere (11–20 km).
/// Surface temperature and pressure default to ISA sea-level values.
///
/// @param altitude_m              Altitude above MSL (m)
/// @param surface_temp_K          Surface temperature (K)    [default 288.15]
/// @param surface_pressure_Pa     Surface pressure (Pa)      [default 101325]
/// @param relative_humidity       Fractional humidity [0,1]  [default 0]
/// @param wind                    Ambient wind               [default zero]
AtmosphericConditions isa_conditions(
    double altitude_m,
    double surface_temp_K       = 288.15,
    double surface_pressure_Pa  = 101325.0,
    double relative_humidity    = 0.0,
    const Wind& wind            = {}) noexcept;

} // namespace ballistics
