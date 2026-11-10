#include "preintegration_core/preintegrator.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <cmath>
#include <cstdio>
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

std::vector<preintegration_core::ImuIncrement> expandIncrements(
    const std::vector<preintegration_core::ImuIncrement>& increments,
    const int subdivisions) {
    std::vector<preintegration_core::ImuIncrement> expanded;
    expanded.reserve(increments.size() * static_cast<std::size_t>(subdivisions));
    for (const auto& increment : increments) {
        preintegration_core::ImuIncrement subdivided = increment;
        subdivided.dt_s =
            increment.dt_s / static_cast<double>(subdivisions);
        subdivided.delta_angle_rad =
            increment.delta_angle_rad / static_cast<double>(subdivisions);
        subdivided.delta_velocity_mps =
            increment.delta_velocity_mps /
            static_cast<double>(subdivisions);
        for (int index = 0; index < subdivisions; ++index) {
            expanded.push_back(subdivided);
        }
    }
    return expanded;
}

}  // namespace

int main() {
    const std::vector<preintegration_core::ImuIncrement> increments =
        makeIncrements();
    constexpr int common_subdivisions = 100;
    const std::vector<preintegration_core::ImuIncrement> expanded_increments =
        expandIncrements(increments, common_subdivisions);

    preintegration_core::PreintegrationOptions options;
    options.enable_coning_sculling = false;
    preintegration_core::Preintegrator core(options);
    for (const auto& increment : expanded_increments) {
        core.add(increment);
    }

    auto parameters =
        gtsam::PreintegrationCombinedParams::MakeSharedD(0.0);
    parameters->setAccelerometerCovariance(
        Eigen::Matrix3d::Identity() * 1e-12);
    parameters->setGyroscopeCovariance(
        Eigen::Matrix3d::Identity() * 1e-12);
    parameters->setIntegrationCovariance(
        Eigen::Matrix3d::Identity() * 1e-12);
    parameters->setBiasAccCovariance(Eigen::Matrix3d::Zero());
    parameters->setBiasOmegaCovariance(Eigen::Matrix3d::Zero());

    const gtsam::imuBias::ConstantBias bias;
    gtsam::PreintegratedCombinedMeasurements gtsam_pim(parameters, bias);
    for (const auto& increment : expanded_increments) {
        gtsam_pim.integrateMeasurement(
            increment.delta_velocity_mps / increment.dt_s,
            increment.delta_angle_rad / increment.dt_s,
            increment.dt_s);
    }

    const auto& core_delta = core.delta();
    const gtsam::Rot3 gtsam_attitude = gtsam_pim.deltaRij();
    const gtsam::Rot3 core_attitude(
        core_delta.delta_q_b0_bk.toRotationMatrix());
    const double attitude_error =
        gtsam_attitude.localCoordinates(core_attitude).norm();
    const double velocity_error =
        (gtsam_pim.deltaVij() - core_delta.delta_v_mps).norm();
    const double position_error =
        (gtsam_pim.deltaPij() - core_delta.delta_p_m).norm();

    check(
        attitude_error < 1e-8,
        "GTSAM relative attitude matches preintegration_core",
        attitude_error);
    check(
        velocity_error < 1e-8,
        "GTSAM relative velocity matches preintegration_core",
        velocity_error);
    check(
        position_error < 1e-8,
        "GTSAM relative position matches preintegration_core",
        position_error);
    check(
        std::abs(gtsam_pim.deltaTij() - core_delta.duration_s) < 1e-12,
        "GTSAM duration matches preintegration_core",
        gtsam_pim.deltaTij() - core_delta.duration_s);

    const gtsam::NavState initial(gtsam::Pose3(), gtsam::Vector3::Zero());
    const gtsam::NavState predicted = gtsam_pim.predict(initial, bias);
    const preintegration_core::NavState composed =
        preintegration_core::compose(
            preintegration_core::NavState{},
            core_delta,
            Eigen::Vector3d::Zero());
    const double predict_position_error =
        (predicted.position() - composed.position_ned_m).norm();
    const double predict_velocity_error =
        (predicted.velocity() - composed.velocity_ned_mps).norm();
    const double predict_attitude_error =
        gtsam::Rot3(predicted.attitude().matrix())
            .localCoordinates(
                gtsam::Rot3(composed.q_bn.toRotationMatrix()))
            .norm();

    check(
        predict_position_error < 1e-8,
        "GTSAM predict position matches compose",
        predict_position_error);
    check(
        predict_velocity_error < 1e-8,
        "GTSAM predict velocity matches compose",
        predict_velocity_error);
    check(
        predict_attitude_error < 1e-8,
        "GTSAM predict attitude matches compose",
        predict_attitude_error);

    if (g_failures == 0) {
        std::printf("preintegration_core_gtsam_regression: all checks passed\n");
        return 0;
    }
    std::printf(
        "preintegration_core_gtsam_regression: %d checks failed\n",
        g_failures);
    return 1;
}
