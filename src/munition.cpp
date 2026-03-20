#include "ballistics/munition.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace ballistics {

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

/// Extract a required double field from a JSON object.
double require_double(const json& obj, const char* field) {
    if (!obj.contains(field))
        throw std::invalid_argument(std::string("Missing required field: ") + field);
    if (!obj[field].is_number())
        throw std::invalid_argument(std::string("Field must be a number: ") + field);
    return obj[field].get<double>();
}

MunitionSpec parse_spec(const json& obj) {
    if (!obj.contains("name") || !obj["name"].is_string())
        throw std::invalid_argument("Munition entry missing 'name' string field");

    MunitionSpec spec;
    spec.name               = obj["name"].get<std::string>();
    spec.mass_kg            = require_double(obj, "mass_kg");
    spec.density_kg_m3      = require_double(obj, "density_kg_m3");
    spec.reference_area_m2  = require_double(obj, "reference_area_m2");
    spec.drag_coefficient   = require_double(obj, "drag_coefficient");
    spec.muzzle_velocity_ms = require_double(obj, "muzzle_velocity_ms");

    // Optional field
    if (obj.contains("diameter_m") && obj["diameter_m"].is_number())
        spec.diameter_m = obj["diameter_m"].get<double>();

    // Basic sanity checks
    if (spec.mass_kg <= 0.0)
        throw std::invalid_argument("mass_kg must be positive for: " + spec.name);
    if (spec.density_kg_m3 <= 0.0)
        throw std::invalid_argument("density_kg_m3 must be positive for: " + spec.name);
    if (spec.reference_area_m2 <= 0.0)
        throw std::invalid_argument("reference_area_m2 must be positive for: " + spec.name);
    if (spec.drag_coefficient < 0.0)
        throw std::invalid_argument("drag_coefficient must be non-negative for: " + spec.name);
    if (spec.muzzle_velocity_ms <= 0.0)
        throw std::invalid_argument("muzzle_velocity_ms must be positive for: " + spec.name);

    return spec;
}

void merge_from_json(const json& root, std::unordered_map<std::string, MunitionSpec>& out) {
    if (!root.contains("munitions") || !root["munitions"].is_array())
        throw std::runtime_error("JSON root must contain a 'munitions' array");

    for (const auto& entry : root["munitions"]) {
        MunitionSpec spec = parse_spec(entry);
        out[spec.name]    = std::move(spec);
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// MunitionLibrary public interface
// ---------------------------------------------------------------------------

void MunitionLibrary::load(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open munitions file: " + json_path);

    json root;
    try {
        file >> root;
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("JSON parse error in ") + json_path + ": " + e.what());
    }

    merge_from_json(root, specs_);
}

void MunitionLibrary::load_from_string(const std::string& json_str) {
    json root;
    try {
        root = json::parse(json_str);
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("JSON parse error: ") + e.what());
    }

    merge_from_json(root, specs_);
}

const MunitionSpec& MunitionLibrary::get(const std::string& name) const {
    auto it = specs_.find(name);
    if (it == specs_.end())
        throw std::out_of_range("Munition not found in library: " + name);
    return it->second;
}

bool MunitionLibrary::contains(const std::string& name) const noexcept {
    return specs_.count(name) > 0;
}

std::vector<std::string> MunitionLibrary::names() const {
    std::vector<std::string> result;
    result.reserve(specs_.size());
    for (const auto& [name, _] : specs_)
        result.push_back(name);
    return result;
}

} // namespace ballistics
