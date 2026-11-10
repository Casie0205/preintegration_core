#include "preintegration_core/preintegrator.hpp"

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

}  // namespace

int main() {
    preintegration_core::PreintegrationOptions options;
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

    if (g_failures == 0) {
        std::printf("preintegration_core_tests: all checks passed\n");
        return 0;
    }
    std::printf("preintegration_core_tests: %d checks failed\n", g_failures);
    return 1;
}
