#pragma once

#include "preintegration_core/runtime_config.hpp"

namespace preintegration_core {

inline RuntimeConfig noEarthConfig() {
    RuntimeConfig config;
    config.enable_earth_rotation = false;
    config.enable_transport_rate = false;
    config.enable_coriolis = false;
    return config;
}

inline RuntimeConfig rotationOnlyConfig() {
    RuntimeConfig config;
    config.enable_transport_rate = false;
    return config;
}

inline RuntimeConfig fullBasicConfig() {
    return RuntimeConfig{};
}

inline RuntimeConfig constantGravityConfig() {
    RuntimeConfig config;
    config.gravity_model = GravityModel::kConstant;
    return config;
}

}  // namespace preintegration_core
