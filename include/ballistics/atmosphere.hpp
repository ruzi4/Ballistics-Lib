#pragma once

namespace ballistics {

/// Constant atmospheric conditions used throughout a simulation run.
/// Air density is fixed for the entire trajectory.
struct AtmosphericConditions {
    double air_density_kg_m3{1.225}; ///< Air density (kg/m³), default ISA sea level
};

} // namespace ballistics
