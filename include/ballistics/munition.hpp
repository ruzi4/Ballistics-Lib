#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace ballistics {

/// Physical specification of a projectile loaded from JSON.
///
/// JSON schema for each entry in the "munitions" array:
/// @code{.json}
/// {
///   "name"              : string  — unique identifier
///   "mass_kg"           : number  — projectile mass (kg)
///   "density_kg_m3"     : number  — material density (kg/m³)
///   "reference_area_m2" : number  — cross-sectional reference area (m²)
///   "drag_coefficient"  : number  — dimensionless Cd
///   "diameter_m"        : number  — characteristic diameter (m) [optional]
/// }
/// @endcode
struct MunitionSpec {
    std::string name;

    double mass_kg{0.0};            ///< Projectile mass (kg)
    double density_kg_m3{0.0};      ///< Material density (kg/m³)
    double reference_area_m2{0.0};  ///< Cross-sectional reference area (m²)
    double drag_coefficient{0.0};   ///< Drag coefficient Cd (dimensionless)
    double diameter_m{0.0};         ///< Characteristic diameter (m)

    /// Ballistic coefficient BC = mass / (Cd × A_ref)  [kg/m²]
    [[nodiscard]] double ballistic_coefficient() const noexcept {
        if (drag_coefficient == 0.0 || reference_area_m2 == 0.0) return 0.0;
        return mass_kg / (drag_coefficient * reference_area_m2);
    }

    /// Volume derived from mass and material density (m³)
    [[nodiscard]] double volume_m3() const noexcept {
        if (density_kg_m3 == 0.0) return 0.0;
        return mass_kg / density_kg_m3;
    }
};

/// In-memory store for munition specifications loaded from a JSON file.
///
/// Thread-safety: construct once and treat as read-only during simulation.
class MunitionLibrary {
public:
    MunitionLibrary() = default;

    /// Load and merge munitions from a JSON file on disk.
    /// @throws std::runtime_error  if the file cannot be opened or parsed.
    /// @throws std::invalid_argument  if a required field is missing or invalid.
    void load(const std::string& json_path);

    /// Load and merge munitions from a JSON string in memory.
    /// @throws std::runtime_error  on parse errors.
    /// @throws std::invalid_argument  if a required field is missing or invalid.
    void load_from_string(const std::string& json_str);

    /// Return a munition by name.
    /// @throws std::out_of_range  if the name is not found.
    [[nodiscard]] const MunitionSpec& get(const std::string& name) const;

    /// Return true if the name exists in the library.
    [[nodiscard]] bool contains(const std::string& name) const noexcept;

    /// Return names of all loaded munitions.
    [[nodiscard]] std::vector<std::string> names() const;

    /// Number of munitions currently stored.
    [[nodiscard]] std::size_t size() const noexcept { return specs_.size(); }

    /// Remove all stored munitions.
    void clear() noexcept { specs_.clear(); }

private:
    std::unordered_map<std::string, MunitionSpec> specs_;
};

} // namespace ballistics
