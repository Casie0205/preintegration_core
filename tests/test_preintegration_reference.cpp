#include "preintegration_core/preintegrator.hpp"

#include <Eigen/Geometry>

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

Eigen::Quaterniond expMap(const Eigen::Vector3d& rotation_vector_rad) {
    const double angle = rotation_vector_rad.norm();
    if (angle < 1e-15) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation_vector_rad / angle));
}

preintegration_core::PreintegratedDelta integrateReference(
    const std::vector<preintegration_core::ImuIncrement>& increments,
    const int subdivisions_per_increment) {
    preintegration_core::PreintegratedDelta reference;
    for (const auto& increment : increments) {
        const double sub_dt =
            increment.dt_s / static_cast<double>(subdivisions_per_increment);
        const Eigen::Vector3d sub_theta =
            increment.delta_angle_rad /
            static_cast<double>(subdivisions_per_increment);
        const Eigen::Vector3d sub_dv =
            increment.delta_velocity_mps /
            static_cast<double>(subdivisions_per_increment);

        for (int index = 0; index < subdivisions_per_increment; ++index) {
            const Eigen::Quaterniond midpoint_attitude =
                (reference.delta_q_b0_bk * expMap(sub_theta * 0.5)).normalized();
            const Eigen::Vector3d dv_b0 =
                midpoint_attitude.toRotationMatrix() * sub_dv;
            reference.delta_p_m +=
                reference.delta_v_mps * sub_dt +
                0.5 * dv_b0 * sub_dt;
            reference.delta_v_mps += dv_b0;
            reference.delta_q_b0_bk =
                (reference.delta_q_b0_bk * expMap(sub_theta)).normalized();
        }
        reference.duration_s += increment.dt_s;
    }
    return reference;
}

}  // namespace

int main() {
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

    preintegration_core::Preintegrator integrator;
    for (const auto& increment : increments) {
        integrator.add(increment);
    }

    const auto reference = integrateReference(increments, 200);
    const auto& actual = integrator.delta();
    const double position_error =
        (actual.delta_p_m - reference.delta_p_m).norm();
    const double velocity_error =
        (actual.delta_v_mps - reference.delta_v_mps).norm();
    const double attitude_error =
        actual.delta_q_b0_bk.angularDistance(reference.delta_q_b0_bk);

    check(
        position_error < 2e-6,
        "relative position agrees with independent fine-step integration",
        position_error);
    check(
        velocity_error < 2e-6,
        "relative velocity agrees with independent fine-step integration",
        velocity_error);
    check(
        attitude_error < 1e-8,
        "relative attitude agrees with independent fine-step integration",
        attitude_error);
    check(
        std::abs(actual.duration_s - reference.duration_s) < 1e-12,
        "reference integration covers the same duration",
        actual.duration_s - reference.duration_s);

    if (g_failures == 0) {
        std::printf("preintegration_core_reference_tests: all checks passed\n");
        return 0;
    }
    std::printf(
        "preintegration_core_reference_tests: %d checks failed\n",
        g_failures);
    return 1;
}
