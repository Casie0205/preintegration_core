#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "preintegration_core/runtime_config.hpp"

namespace preintegration_core {
namespace detail {

inline YAML::Node runtimeNode(const YAML::Node& root) {
    return root["runtime"] ? root["runtime"] : root;
}

template <typename T>
inline T readValue(
    const YAML::Node& node,
    const char* key,
    const T& default_value) {
    return node[key] ? node[key].as<T>() : default_value;
}

inline GravityModel parseGravityModel(const YAML::Node& node) {
    if (!node["gravity_model"]) {
        return GravityModel::kNormalGravity;
    }
    std::string value = node["gravity_model"].as<std::string>();
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    if (value == "normal" || value == "normal_gravity" ||
        value == "knormalgravity") {
        return GravityModel::kNormalGravity;
    }
    if (value == "constant" || value == "kconstant") {
        return GravityModel::kConstant;
    }
    throw std::invalid_argument(
        "gravity_model must be 'normal' or 'constant'");
}

}  // namespace detail

inline RuntimeConfig runtimeConfigFromYaml(const YAML::Node& root) {
    const YAML::Node node = detail::runtimeNode(root);
    RuntimeConfig config;
    config.enable_earth_rotation =
        detail::readValue(node, "enable_earth_rotation",
                          config.enable_earth_rotation);
    config.enable_transport_rate =
        detail::readValue(node, "enable_transport_rate",
                          config.enable_transport_rate);
    config.enable_coriolis =
        detail::readValue(node, "enable_coriolis",
                          config.enable_coriolis);
    config.enable_ellipsoid_radii =
        detail::readValue(node, "enable_ellipsoid_radii",
                          config.enable_ellipsoid_radii);
    config.gravity_model = detail::parseGravityModel(node);
    config.earth_rotation_rate_radps =
        detail::readValue(node, "earth_rotation_rate_radps",
                          config.earth_rotation_rate_radps);
    config.constant_gravity_mps2 =
        detail::readValue(node, "constant_gravity_mps2",
                          config.constant_gravity_mps2);
    config.semi_major_axis_m =
        detail::readValue(node, "semi_major_axis_m",
                          config.semi_major_axis_m);
    config.eccentricity_squared =
        detail::readValue(node, "eccentricity_squared",
                          config.eccentricity_squared);
    config.fallback_sphere_radius_m =
        detail::readValue(node, "fallback_sphere_radius_m",
                          config.fallback_sphere_radius_m);
    config.enable_coning_sculling =
        detail::readValue(node, "enable_coning_sculling",
                          config.enable_coning_sculling);
    config.equal_interval_tolerance_s =
        detail::readValue(node, "equal_interval_tolerance_s",
                          config.equal_interval_tolerance_s);
    return config;
}

inline RuntimeConfig loadRuntimeConfigFromString(const std::string& yaml_text) {
    try {
        return runtimeConfigFromYaml(YAML::Load(yaml_text));
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error(
            std::string("failed to parse runtime YAML: ") + exception.what());
    }
}

inline RuntimeConfig loadRuntimeConfigFile(const std::string& path) {
    try {
        return runtimeConfigFromYaml(YAML::LoadFile(path));
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error(
            std::string("failed to load runtime YAML '") + path +
            "': " + exception.what());
    }
}

}  // namespace preintegration_core
