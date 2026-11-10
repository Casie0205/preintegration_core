#include "preintegration_core/runtime_config_yaml.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

int g_failures = 0;

void check(const bool condition, const char* name) {
    if (condition) {
        std::printf("[ok]   %s\n", name);
    } else {
        std::printf("[FAIL] %s\n", name);
        ++g_failures;
    }
}

}  // namespace

int main() {
    const preintegration_core::RuntimeConfig file_config =
        preintegration_core::loadRuntimeConfigFile(RUNTIME_CONFIG_FILE);
    check(
        file_config.enable_earth_rotation &&
            file_config.enable_transport_rate &&
            file_config.enable_coriolis &&
            file_config.enable_ellipsoid_radii &&
            file_config.enable_coning_sculling &&
            file_config.gravity_model ==
                preintegration_core::GravityModel::kNormalGravity,
        "runtime.yaml loads the default switches");
    check(
        std::abs(file_config.equal_interval_tolerance_s - 1e-9) < 1e-20,
        "runtime.yaml loads numeric values");

    const preintegration_core::RuntimeConfig override_config =
        preintegration_core::loadRuntimeConfigFromString(R"(
runtime:
  enable_earth_rotation: false
  enable_transport_rate: false
  enable_coriolis: false
  enable_ellipsoid_radii: false
  gravity_model: constant
  earth_rotation_rate_radps: 1.0e-5
  constant_gravity_mps2: 9.81
  semi_major_axis_m: 6378000.0
  eccentricity_squared: 0.0067
  fallback_sphere_radius_m: 6370000.0
  enable_coning_sculling: false
  equal_interval_tolerance_s: 2.0e-9
)");
    check(
        !override_config.enable_earth_rotation &&
            !override_config.enable_transport_rate &&
            !override_config.enable_coriolis &&
            !override_config.enable_ellipsoid_radii &&
            !override_config.enable_coning_sculling &&
            override_config.gravity_model ==
                preintegration_core::GravityModel::kConstant,
        "YAML overrides update switch fields");
    check(
        std::abs(override_config.earth_rotation_rate_radps - 1e-5) < 1e-18 &&
            std::abs(override_config.constant_gravity_mps2 - 9.81) < 1e-15 &&
            std::abs(override_config.semi_major_axis_m - 6378000.0) < 1e-9 &&
            std::abs(override_config.eccentricity_squared - 0.0067) < 1e-15 &&
            std::abs(override_config.fallback_sphere_radius_m - 6370000.0) <
                1e-9 &&
            std::abs(override_config.equal_interval_tolerance_s - 2e-9) <
                1e-20,
        "YAML overrides update numeric fields");

    const preintegration_core::RuntimeConfig partial =
        preintegration_core::loadRuntimeConfigFromString(
            "enable_earth_rotation: false\n");
    check(
        !partial.enable_earth_rotation &&
            partial.enable_transport_rate &&
            partial.enable_coriolis,
        "missing YAML fields retain defaults");

    bool invalid_gravity_thrown = false;
    try {
        preintegration_core::loadRuntimeConfigFromString(
            "gravity_model: unsupported\n");
    } catch (const std::invalid_argument&) {
        invalid_gravity_thrown = true;
    }
    check(invalid_gravity_thrown, "invalid gravity model is rejected");

    if (g_failures == 0) {
        std::printf("preintegration_core_yaml_config_tests: all checks passed\n");
        return 0;
    }
    std::printf(
        "preintegration_core_yaml_config_tests: %d checks failed\n",
        g_failures);
    return 1;
}
