#include "preintegration_core/preintegrator.hpp"
#include "preintegration_core/presets.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace {

int g_failures = 0;

void check(const bool condition, const char* name, const double value = 0.0) {
    if (condition) {
        std::printf("[ok]   %s\n", name);
    } else {
        std::printf("[FAIL] %s (%g)\n", name, value);
        ++g_failures;
    }
}

bool near(const double value, const double expected, const double tolerance) {
    return std::abs(value - expected) <= tolerance;
}

Eigen::Quaterniond expMap(const Eigen::Vector3d& rotation_vector_rad) {
    const double angle = rotation_vector_rad.norm();
    if (angle < 1e-15) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation_vector_rad / angle));
}

}  // namespace

int main() {
    preintegration_core::PreintegrationOptions options;
    const preintegration_core::RuntimeConfig no_earth_preset =
        preintegration_core::noEarthConfig();
    const preintegration_core::RuntimeConfig rotation_only_preset =
        preintegration_core::rotationOnlyConfig();
    check(
        !no_earth_preset.enable_earth_rotation &&
            !no_earth_preset.enable_transport_rate &&
            !no_earth_preset.enable_coriolis &&
            rotation_only_preset.enable_earth_rotation &&
            !rotation_only_preset.enable_transport_rate &&
            rotation_only_preset.enable_coriolis,
        "runtime presets expose consistent experiment modes");
    check(
        options.enable_earth_rotation &&
            options.enable_transport_rate &&
            options.enable_coriolis &&
            options.enable_ellipsoid_radii &&
            options.enable_coning_sculling &&
            options.gravity_model ==
                preintegration_core::GravityModel::kNormalGravity,
        "runtime configuration defaults match the shared schema");
    preintegration_core::ImuIncrement acceleration;
    acceleration.dt_s = 0.001;
    acceleration.delta_velocity_mps = Eigen::Vector3d(10.0, -2.0, 0.5) * acceleration.dt_s;

    preintegration_core::Preintegrator accelerator(options);
    for (int index = 0; index < 1000; ++index) {
        accelerator.add(acceleration);
    }
    const auto& acceleration_delta = accelerator.delta();
    check(
        accelerator.sampleCount() == 1000,
        "sample count tracks processed increments",
        static_cast<double>(accelerator.sampleCount()));
    check(
        near(acceleration_delta.duration_s, 1.0, 1e-12),
        "duration accumulates IMU intervals",
        acceleration_delta.duration_s);
    check(
        near(acceleration_delta.delta_v_mps.x(), 10.0, 1e-10) &&
            near(acceleration_delta.delta_v_mps.y(), -2.0, 1e-10) &&
            near(acceleration_delta.delta_v_mps.z(), 0.5, 1e-10),
        "constant acceleration integrates to velocity");
    check(
        near(acceleration_delta.delta_p_m.x(), 5.0, 1e-8) &&
            near(acceleration_delta.delta_p_m.y(), -1.0, 1e-8) &&
            near(acceleration_delta.delta_p_m.z(), 0.25, 1e-8),
        "constant acceleration integrates to position");

    preintegration_core::ImuIncrement rotation;
    rotation.dt_s = 0.001;
    rotation.delta_angle_rad = Eigen::Vector3d(0.0, 0.0, 0.25) * rotation.dt_s;
    preintegration_core::Preintegrator rotator(options);
    for (int index = 0; index < 1000; ++index) {
        rotator.add(rotation);
    }
    const double yaw = std::atan2(
        rotator.delta().delta_q_b0_bk.toRotationMatrix()(1, 0),
        rotator.delta().delta_q_b0_bk.toRotationMatrix()(0, 0));
    check(
        near(yaw, 0.25, 1e-10),
        "constant angular rate integrates to an attitude increment",
        yaw);

    preintegration_core::NavState initial;
    initial.time_s = 2.0;
    initial.position_ned_m = Eigen::Vector3d(12.0, -3.0, 5.0);
    initial.velocity_ned_mps = Eigen::Vector3d(1.5, -0.5, 0.2);
    initial.q_bn = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d gravity(0.0, 0.0, 9.80665);
    const preintegration_core::NavState composed =
        preintegration_core::compose(
            initial, acceleration_delta, gravity);
    const double cos_yaw = std::cos(0.3);
    const double sin_yaw = std::sin(0.3);
    check(
        near(composed.time_s, 3.0, 1e-12),
        "compose advances time");
    check(
        near(
            composed.velocity_ned_mps.x(),
            initial.velocity_ned_mps.x() + cos_yaw * 10.0 + sin_yaw * 2.0,
            1e-9) &&
            near(
                composed.velocity_ned_mps.y(),
                initial.velocity_ned_mps.y() + sin_yaw * 10.0 -
                    cos_yaw * 2.0,
                1e-9) &&
            near(
                composed.velocity_ned_mps.z(),
                initial.velocity_ned_mps.z() + 9.80665 + 0.5,
                1e-9),
        "compose rotates body-frame velocity into navigation coordinates",
        composed.velocity_ned_mps.x());
    check(
        near(
            composed.position_ned_m.x(),
            initial.position_ned_m.x() + initial.velocity_ned_mps.x() +
                cos_yaw * 5.0 + sin_yaw * 1.0,
            1e-8) &&
            near(
                composed.position_ned_m.y(),
                initial.position_ned_m.y() + initial.velocity_ned_mps.y() +
                    sin_yaw * 5.0 - cos_yaw * 1.0,
                1e-8) &&
            near(
                composed.position_ned_m.z(),
                initial.position_ned_m.z() + initial.velocity_ned_mps.z() +
                    0.25 + 0.5 * 9.80665,
                1e-8),
        "compose rotates body-frame position into navigation coordinates",
        composed.position_ned_m.x());

    preintegration_core::NavState attitude_initial;
    const preintegration_core::NavState attitude_end =
        preintegration_core::compose(
            attitude_initial, rotator.delta(), Eigen::Vector3d::Zero());
    const Eigen::Quaterniond expected_attitude(
        Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
    check(
        attitude_end.q_bn.angularDistance(expected_attitude) < 1e-10,
        "compose multiplies initial attitude on the left of DeltaR");
    check(
        std::abs(rotator.delta().delta_q_b0_bk.norm() - 1.0) < 1e-14,
        "relative attitude remains normalized",
        rotator.delta().delta_q_b0_bk.norm());

    preintegration_core::Preintegrator first_sample(options);
    preintegration_core::ImuIncrement first_increment;
    first_increment.dt_s = 0.001;
    first_increment.delta_angle_rad =
        Eigen::Vector3d(0.0, 0.0, 0.2 * first_increment.dt_s);
    first_increment.delta_velocity_mps =
        Eigen::Vector3d(1.0, 0.0, 0.0) * first_increment.dt_s;
    first_sample.add(first_increment);
    check(
        near(first_sample.delta().delta_v_mps.x(), 0.001, 1e-15) &&
            near(first_sample.delta().delta_v_mps.y(), 1e-7, 1e-15),
        "first sample still applies the current-sample sculling term",
        first_sample.delta().delta_v_mps.y());

    rotator.clear();
    check(
        rotator.sampleCount() == 0 &&
            near(rotator.delta().duration_s, 0.0, 0.0) &&
            near(rotator.delta().delta_q_b0_bk.w(), 1.0, 1e-15),
        "clear resets the preintegration",
        rotator.sampleCount());

    bool invalid_dt_thrown = false;
    try {
        preintegration_core::ImuIncrement invalid;
        rotator.add(invalid);
    } catch (const std::invalid_argument&) {
        invalid_dt_thrown = true;
    }
    check(invalid_dt_thrown, "non-positive dt is rejected");

    bool unequal_dt_thrown = false;
    try {
        preintegration_core::Preintegrator unequal(options);
        preintegration_core::ImuIncrement first;
        first.dt_s = 0.001;
        preintegration_core::ImuIncrement second;
        second.dt_s = 0.002;
        unequal.add(first);
        unequal.add(second);
    } catch (const std::invalid_argument&) {
        unequal_dt_thrown = true;
    }
    check(unequal_dt_thrown, "unequal dt is rejected for two-sample corrections");

    bool nonfinite_thrown = false;
    try {
        preintegration_core::ImuIncrement nonfinite;
        nonfinite.dt_s = 0.001;
        nonfinite.delta_velocity_mps.x() =
            std::numeric_limits<double>::quiet_NaN();
        rotator.add(nonfinite);
    } catch (const std::invalid_argument&) {
        nonfinite_thrown = true;
    }
    check(nonfinite_thrown, "non-finite increment values are rejected");

    bool invalid_tolerance_thrown = false;
    try {
        preintegration_core::PreintegrationOptions invalid_options;
        invalid_options.equal_interval_tolerance_s = -1.0;
        preintegration_core::Preintegrator invalid_tolerance(invalid_options);
        invalid_tolerance.add(acceleration);
    } catch (const std::invalid_argument&) {
        invalid_tolerance_thrown = true;
    }
    check(invalid_tolerance_thrown, "invalid equal-interval tolerance is rejected");

    preintegration_core::PreintegrationOptions disabled_options;
    disabled_options.enable_coning_sculling = false;
    preintegration_core::Preintegrator disabled_sculling(disabled_options);
    disabled_sculling.add(first_increment);
    check(
        near(disabled_sculling.delta().delta_v_mps.x(), 0.001, 1e-15) &&
            near(disabled_sculling.delta().delta_v_mps.y(), 0.0, 1e-15),
        "coning/sculling disable switch removes the cross term");

    bool invalid_compose_attitude_thrown = false;
    try {
        preintegration_core::NavState invalid_attitude;
        invalid_attitude.q_bn.coeffs().setZero();
        preintegration_core::compose(
            invalid_attitude,
            preintegration_core::PreintegratedDelta{},
            Eigen::Vector3d::Zero());
    } catch (const std::invalid_argument&) {
        invalid_compose_attitude_thrown = true;
    }
    check(
        invalid_compose_attitude_thrown,
        "compose rejects a zero initial attitude quaternion");

    bool negative_duration_thrown = false;
    try {
        preintegration_core::PreintegratedDelta invalid_delta;
        invalid_delta.duration_s = -1.0;
        preintegration_core::compose(
            preintegration_core::NavState{},
            invalid_delta,
            Eigen::Vector3d::Zero());
    } catch (const std::invalid_argument&) {
        negative_duration_thrown = true;
    }
    check(negative_duration_thrown, "compose rejects negative duration");

    preintegration_core::NavState environment_initial;
    environment_initial.velocity_ned_mps = Eigen::Vector3d(1.0, 2.0, 3.0);
    preintegration_core::PreintegratedDelta unit_delta;
    unit_delta.duration_s = 1.0;
    preintegration_core::EarthEnvironment environment;
    environment.gravity_ned_mps2 = Eigen::Vector3d(0.0, 0.0, 9.8);
    environment.omega_ie_ned_radps =
        Eigen::Vector3d(7.0e-5, 0.0, -5.0e-5);
    environment.omega_en_ned_radps =
        Eigen::Vector3d(0.1, 0.0, 0.05);

    preintegration_core::RuntimeConfig inert_options;
    inert_options.enable_earth_rotation = false;
    inert_options.enable_transport_rate = false;
    inert_options.enable_coriolis = false;
    const preintegration_core::NavState inert_end =
        preintegration_core::composeWithEnvironment(
            environment_initial, unit_delta, environment, inert_options);
    const preintegration_core::NavState inert_reference =
        preintegration_core::compose(
            environment_initial, unit_delta, environment.gravity_ned_mps2);
    check(
        (inert_end.velocity_ned_mps - inert_reference.velocity_ned_mps).norm() <
                1e-14 &&
            (inert_end.position_ned_m - inert_reference.position_ned_m).norm() <
                1e-14 &&
            inert_end.q_bn.angularDistance(inert_reference.q_bn) < 1e-14,
        "disabled Earth switches reproduce the inert composition");

    const preintegration_core::NavState earth_end =
        preintegration_core::composeWithEnvironment(
            environment_initial,
            unit_delta,
            environment,
            preintegration_core::RuntimeConfig{});
    const Eigen::Vector3d rate_cross =
        environment.omega_en_ned_radps +
        2.0 * environment.omega_ie_ned_radps;
    const Eigen::Vector3d specific_force =
        environment.gravity_ned_mps2 -
        rate_cross.cross(environment_initial.velocity_ned_mps);
    const Eigen::Vector3d expected_velocity =
        environment_initial.velocity_ned_mps + specific_force;
    const Eigen::Vector3d expected_position =
        environment_initial.position_ned_m +
        environment_initial.velocity_ned_mps +
        0.5 * specific_force;
    const Eigen::Quaterniond expected_earth_attitude = expMap(
        -(environment.omega_ie_ned_radps +
          environment.omega_en_ned_radps));
    check(
        (earth_end.velocity_ned_mps - expected_velocity).norm() < 1e-12,
        "Earth composition applies Coriolis and transport terms");
    check(
        earth_end.q_bn.angularDistance(expected_earth_attitude) < 1e-12,
        "Earth composition applies navigation-frame rotation");
    check(
        (earth_end.position_ned_m - expected_position).norm() < 1e-12,
        "Earth composition applies the selected acceleration to position");

    constexpr double latitude_rad =
        30.5 * 3.14159265358979323846 / 180.0;
    const preintegration_core::EarthEnvironment generated_environment =
        preintegration_core::makeEarthEnvironment(
            latitude_rad,
            20.0,
            Eigen::Vector3d(5.0, 2.0, 0.0),
            preintegration_core::RuntimeConfig{});
    check(
        std::abs(generated_environment.gravity_ned_mps2.z() - 9.7935) <
            2e-3,
        "generated normal gravity is plausible",
        generated_environment.gravity_ned_mps2.z());
    check(
        generated_environment.omega_ie_ned_radps.x() > 0.0 &&
            generated_environment.omega_ie_ned_radps.z() < 0.0,
        "generated Earth rate uses the NED convention");
    check(
        generated_environment.omega_en_ned_radps.norm() > 0.0,
        "generated transport rate is nonzero for horizontal velocity");

    preintegration_core::RuntimeConfig no_earth_config;
    no_earth_config.enable_earth_rotation = false;
    no_earth_config.enable_transport_rate = false;
    const preintegration_core::EarthEnvironment generated_no_earth =
        preintegration_core::makeEarthEnvironment(
            latitude_rad,
            20.0,
            Eigen::Vector3d(5.0, 2.0, 0.0),
            no_earth_config);
    check(
        generated_no_earth.omega_ie_ned_radps.isZero(0.0) &&
            generated_no_earth.omega_en_ned_radps.isZero(0.0),
        "environment builder honors Earth rotation switches");

    if (g_failures == 0) {
        std::printf("preintegration_core_tests: all checks passed\n");
        return 0;
    }
    std::printf("preintegration_core_tests: %d checks failed\n", g_failures);
    return 1;
}
