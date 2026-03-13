#pragma once

#include "math/vec3.hpp"

namespace ballistics {

/// Wind vector in world-space coordinates (m/s).
/// Coordinate convention: x = East, y = North, z = Up.
struct Wind {
    Vec3 velocity_ms;  ///< World-space wind velocity (m/s)
};

/// Constant atmospheric conditions used throughout a simulation run.
/// Air density is fixed for the entire trajectory; no altitude variation.
struct AtmosphericConditions {
    double air_density_kg_m3{1.225}; ///< Air density (kg/m³), default ISA sea level
    Wind   wind{};                   ///< Ambient wind
};

} // namespace ballistics
