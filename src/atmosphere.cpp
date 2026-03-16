#include "ballistics/atmosphere.hpp"

#include <cmath>

namespace ballistics {

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
namespace {
constexpr double kMolarMassDryAir = 0.0289644; // kg/mol
constexpr double kUniversalGas    = 8.31446;   // J/(mol·K)
constexpr double kGravity         = 9.80665;   // m/s²
constexpr double kLapseRate       = 0.0065;    // K/m (troposphere)
constexpr double kTropopause      = 11000.0;   // m
constexpr double kStratoPause     = 20000.0;   // m (upper limit of model)

// ISA reference values at 11 km
constexpr double kT11 = 216.65;  // K
constexpr double kP11 = 22632.1; // Pa

// Exponent for pressure-altitude relationship in the troposphere
constexpr double kTropoExp = kGravity * kMolarMassDryAir / (kUniversalGas * kLapseRate); // ~5.2561
} // anonymous namespace

// ---------------------------------------------------------------------------
// Air density
// ---------------------------------------------------------------------------

double
compute_air_density(double temperature_K, double pressure_Pa, double relative_humidity) noexcept {
    // Water vapour pressure via the Magnus formula (Pa)
    // P_sat = 611.657 * exp(17.368*(T-273.15)/(T-273.15+238.88))   [Alduchov 1996]
    const double T_C   = temperature_K - 273.15;
    const double P_sat = 611.657 * std::exp(17.368 * T_C / (T_C + 238.88));
    const double P_v   = relative_humidity * P_sat; // partial vapour pressure

    // Density of moist air (Tetens / Dalton):
    // rho = (P_d * M_d + P_v * M_v) / (R * T)
    // where M_v = 0.018016 kg/mol (water), P_d = P - P_v
    const double P_d = pressure_Pa - P_v;
    return (P_d * kMolarMassDryAir + P_v * 0.018016) / (kUniversalGas * temperature_K);
}

// ---------------------------------------------------------------------------
// AtmosphericConditions member
// ---------------------------------------------------------------------------

void AtmosphericConditions::recompute_density() noexcept {
    air_density_kg_m3 = compute_air_density(temperature_K, pressure_Pa, relative_humidity);
}

// ---------------------------------------------------------------------------
// ISA model
// ---------------------------------------------------------------------------

AtmosphericConditions isa_conditions(double      altitude_m,
                                     double      surface_temp_K,
                                     double      surface_pressure_Pa,
                                     double      relative_humidity,
                                     const Wind& wind) noexcept {
    AtmosphericConditions cond;
    cond.relative_humidity = relative_humidity;
    cond.wind              = wind;

    if (altitude_m <= kTropopause) {
        // Troposphere: linear temperature lapse
        const double T     = surface_temp_K - kLapseRate * altitude_m;
        const double P     = surface_pressure_Pa * std::pow(T / surface_temp_K, kTropoExp);
        cond.temperature_K = T;
        cond.pressure_Pa   = P;
    } else if (altitude_m <= kStratoPause) {
        // Lower stratosphere: isothermal at 216.65 K
        // Adjust kP11 for non-standard surface conditions
        const double T11_adj = surface_temp_K - kLapseRate * kTropopause;
        const double P11_adj = surface_pressure_Pa * std::pow(T11_adj / surface_temp_K, kTropoExp);

        const double T     = kT11;
        const double P     = P11_adj * std::exp(-kGravity * kMolarMassDryAir *
                                            (altitude_m - kTropopause) / (kUniversalGas * kT11));
        cond.temperature_K = T;
        cond.pressure_Pa   = P;
    } else {
        // Beyond model range — clamp to stratopause value
        const double T11_adj = surface_temp_K - kLapseRate * kTropopause;
        const double P11_adj = surface_pressure_Pa * std::pow(T11_adj / surface_temp_K, kTropoExp);
        cond.temperature_K   = kT11;
        cond.pressure_Pa =
            P11_adj * std::exp(-kGravity * kMolarMassDryAir * (kStratoPause - kTropopause) /
                               (kUniversalGas * kT11));
    }

    cond.recompute_density();
    return cond;
}

} // namespace ballistics
