#include "preintegration_core/preintegrator.hpp"

#include "preintegration_normal.h"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

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

std::vector<preintegration_core::ImuIncrement> makeIncrements() {
    constexpr int sample_count = 1000;
    constexpr double dt = 0.001;
    std::vector<preintegration_core::ImuIncrement> increments;
    increments.reserve(sample_count);
    for (int index = 0; index < sample_count; ++index) {
        const double time =
            (static_cast<double>(index) + 0.5) * dt;
        preintegration_core::ImuIncrement increment;
        increment.dt_s = dt;
        increment.delta_angle_rad =
            Eigen::Vector3d(
                0.20 * std::sin(3.0 * time),
                -0.15 * std::cos(2.0 * time),
                0.40 + 0.10 * std::sin(time)) *
            dt;
        increment.delta_velocity_mps =
            Eigen::Vector3d(
                2.0 + 0.5 * std::sin(5.0 * time),
                -1.0 + 0.3 * std::cos(4.0 * time),
                0.2 * std::sin(2.0 * time)) *
            dt;
        increments.push_back(increment);
    }
    return increments;
}

IMU makeObImu(
    const preintegration_core::ImuIncrement& increment,
    const double time_s) {
    IMU imu;
    imu.time = time_s;
    imu.dt = increment.dt_s;
    imu.dtheta = increment.delta_angle_rad;
    imu.dvel = increment.delta_velocity_mps;
    imu.odovel = 0.0;
    return imu;
}

}  // namespace

int main() {
    const std::vector<preintegration_core::ImuIncrement> increments =
        makeIncrements();
    if (increments.size() < 2U) {
        return 1;
    }

    preintegration_core::Preintegrator core;
    core.seedPreviousIncrement(increments.front());
    for (std::size_t index = 1; index < increments.size(); ++index) {
        core.add(increments[index]);
    }

    auto parameters = std::make_shared<IntegrationParameters>();
    parameters->gravity = 0.0;
    parameters->gyr_arw = 1e-8;
    parameters->acc_vrw = 1e-8;
    parameters->gyr_bias_std = 1e-8;
    parameters->acc_bias_std = 1e-8;
    parameters->gyr_scale_std = 1e-8;
    parameters->acc_scale_std = 1e-8;
    parameters->corr_time = 3600.0;
    parameters->odo_std = Eigen::Vector3d::Constant(1e-8);
    parameters->odo_srw = 1e-8;
    parameters->abv = Eigen::Vector3d::Zero();
    parameters->lodo = Eigen::Vector3d::Zero();
    parameters->station = Eigen::Vector3d::Zero();

    IntegrationState initial_state;
    initial_state.time = 0.0;
    initial_state.p = Eigen::Vector3d::Zero();
    initial_state.q = Eigen::Quaterniond::Identity();
    initial_state.v = Eigen::Vector3d::Zero();
    initial_state.bg = Eigen::Vector3d::Zero();
    initial_state.ba = Eigen::Vector3d::Zero();

    auto ob_preintegration = std::make_shared<PreintegrationNormal>(
        parameters,
        makeObImu(increments.front(), 0.0),
        initial_state);
    double time_s = 0.0;
    for (std::size_t index = 1; index < increments.size(); ++index) {
        time_s += increments[index].dt_s;
        ob_preintegration->addNewImu(
            makeObImu(increments[index], time_s));
    }

    const auto& core_delta = core.delta();
    const IntegrationState& ob_delta = ob_preintegration->deltaState();
    const double attitude_error =
        core_delta.delta_q_b0_bk.angularDistance(ob_delta.q);
    const double velocity_error =
        (core_delta.delta_v_mps - ob_delta.v).norm();
    const double position_error =
        (core_delta.delta_p_m - ob_delta.p).norm();

    check(
        attitude_error < 1e-12,
        "OB_GINS relative attitude matches preintegration_core",
        attitude_error);
    check(
        velocity_error < 1e-12,
        "OB_GINS relative velocity matches preintegration_core",
        velocity_error);
    check(
        position_error < 1e-12,
        "OB_GINS relative position matches preintegration_core",
        position_error);
    check(
        std::abs(ob_preintegration->deltaTime() - core_delta.duration_s) <
            1e-12,
        "OB_GINS duration matches preintegration_core",
        ob_preintegration->deltaTime() - core_delta.duration_s);

    if (g_failures == 0) {
        std::printf("preintegration_core_ob_gins_regression: all checks passed\n");
        return 0;
    }
    std::printf(
        "preintegration_core_ob_gins_regression: %d checks failed\n",
        g_failures);
    return 1;
}
