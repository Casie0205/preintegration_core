#include "preintegration_core/preintegrator.hpp"

#include <cmath>
#include <stdexcept>

namespace preintegration_core {
namespace {

Eigen::Quaterniond expMap(const Eigen::Vector3d& rotation_vector_rad) {
    const double angle = rotation_vector_rad.norm();
    if (angle < 1e-15) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation_vector_rad / angle));
}

void validateIncrement(const ImuIncrement& increment) {
    if (!std::isfinite(increment.dt_s) || increment.dt_s <= 0.0) {
        throw std::invalid_argument("IMU increment dt must be finite and positive");
    }
    if (!increment.delta_angle_rad.allFinite() ||
        !increment.delta_velocity_mps.allFinite()) {
        throw std::invalid_argument("IMU increment values must be finite");
    }
}

}  // namespace

Preintegrator::Preintegrator(PreintegrationOptions options)
    : options_(options) {
    clear();
}

void Preintegrator::clear() {
    delta_ = PreintegratedDelta{};
    previous_increment_ = ImuIncrement{};
    has_previous_increment_ = false;
    sample_count_ = 0;
}

void Preintegrator::add(const ImuIncrement& increment) {
    validateIncrement(increment);
    if (!std::isfinite(options_.equal_interval_tolerance_s) ||
        options_.equal_interval_tolerance_s < 0.0) {
        throw std::invalid_argument("equal-interval tolerance must be finite and nonnegative");
    }
    if (options_.enable_coning_sculling &&
        has_previous_increment_ &&
        std::abs(previous_increment_.dt_s - increment.dt_s) >
            options_.equal_interval_tolerance_s) {
        throw std::invalid_argument(
            "adjacent-sample coning/sculling requires equal IMU intervals");
    }

    const Eigen::Vector3d d_theta = increment.delta_angle_rad;
    const Eigen::Vector3d d_v = increment.delta_velocity_mps;

    Eigen::Vector3d d_v_body = d_v;
    Eigen::Vector3d d_theta_body = d_theta;
    if (options_.enable_coning_sculling) {
        d_v_body += 0.5 * d_theta.cross(d_v);
    }
    if (options_.enable_coning_sculling && has_previous_increment_) {
        d_v_body +=
            (1.0 / 12.0) *
                (previous_increment_.delta_angle_rad.cross(d_v) +
                 previous_increment_.delta_velocity_mps.cross(d_theta));
        d_theta_body +=
            (1.0 / 12.0) *
            previous_increment_.delta_angle_rad.cross(d_theta);
    }

    const Eigen::Matrix3d c_b0_bk =
        delta_.delta_q_b0_bk.toRotationMatrix();
    const Eigen::Vector3d d_v_b0 = c_b0_bk * d_v_body;
    delta_.delta_p_m +=
        increment.dt_s * delta_.delta_v_mps +
        0.5 * increment.dt_s * d_v_b0;
    delta_.delta_v_mps += d_v_b0;
    delta_.delta_q_b0_bk =
        (delta_.delta_q_b0_bk * expMap(d_theta_body)).normalized();
    delta_.duration_s += increment.dt_s;

    if (!delta_.delta_q_b0_bk.coeffs().allFinite() ||
        !delta_.delta_v_mps.allFinite() ||
        !delta_.delta_p_m.allFinite() ||
        !std::isfinite(delta_.duration_s)) {
        throw std::runtime_error("preintegration produced a non-finite delta");
    }

    previous_increment_ = increment;
    has_previous_increment_ = true;
    ++sample_count_;
}

NavState compose(
    const NavState& initial,
    const PreintegratedDelta& delta,
    const Eigen::Vector3d& gravity_ned_mps2) {
    if (!initial.position_ned_m.allFinite() ||
        !initial.velocity_ned_mps.allFinite() ||
        !initial.q_bn.coeffs().allFinite() ||
        initial.q_bn.norm() < 1e-12 ||
        !std::isfinite(initial.time_s) ||
        !std::isfinite(delta.duration_s) ||
        delta.duration_s < 0.0 ||
        !delta.delta_q_b0_bk.coeffs().allFinite() ||
        delta.delta_q_b0_bk.norm() < 1e-12 ||
        !delta.delta_v_mps.allFinite() ||
        !delta.delta_p_m.allFinite() ||
        !gravity_ned_mps2.allFinite()) {
        throw std::invalid_argument("compose arguments must be finite and valid");
    }

    const Eigen::Quaterniond initial_attitude =
        initial.q_bn.normalized();
    const Eigen::Matrix3d c_bn = initial_attitude.toRotationMatrix();

    NavState end = initial;
    end.time_s += delta.duration_s;
    end.q_bn =
        (initial_attitude * delta.delta_q_b0_bk).normalized();
    end.velocity_ned_mps =
        initial.velocity_ned_mps +
        gravity_ned_mps2 * delta.duration_s +
        c_bn * delta.delta_v_mps;
    end.position_ned_m =
        initial.position_ned_m +
        initial.velocity_ned_mps * delta.duration_s +
        0.5 * gravity_ned_mps2 * delta.duration_s * delta.duration_s +
        c_bn * delta.delta_p_m;
    return end;
}

}  // namespace preintegration_core
