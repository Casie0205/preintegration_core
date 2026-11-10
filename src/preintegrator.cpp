#include "preintegration_core/preintegrator.hpp"

#include <cmath>
#include <stdexcept>

namespace preintegration_core {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi * 0.5;

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

void validateOptions(const RuntimeConfig& options) {
    if (!std::isfinite(options.equal_interval_tolerance_s) ||
        options.equal_interval_tolerance_s < 0.0) {
        throw std::invalid_argument(
            "equal-interval tolerance must be finite and nonnegative");
    }
    if (!std::isfinite(options.earth_rotation_rate_radps) ||
        options.earth_rotation_rate_radps < 0.0) {
        throw std::invalid_argument(
            "earth rotation rate must be finite and nonnegative");
    }
    if (!std::isfinite(options.constant_gravity_mps2) ||
        options.constant_gravity_mps2 <= 0.0 ||
        !std::isfinite(options.semi_major_axis_m) ||
        options.semi_major_axis_m <= 0.0 ||
        !std::isfinite(options.eccentricity_squared) ||
        options.eccentricity_squared < 0.0 ||
        options.eccentricity_squared >= 1.0 ||
        !std::isfinite(options.fallback_sphere_radius_m) ||
        options.fallback_sphere_radius_m <= 0.0) {
        throw std::invalid_argument("runtime earth constants are invalid");
    }
}

Eigen::Vector2d meridianPrimeVerticalRadii(
    const double latitude_rad,
    const RuntimeConfig& options) {
    if (!options.enable_ellipsoid_radii) {
        return Eigen::Vector2d::Constant(options.fallback_sphere_radius_m);
    }
    const double sin_latitude = std::sin(latitude_rad);
    const double denominator =
        1.0 - options.eccentricity_squared *
                  sin_latitude * sin_latitude;
    const double sqrt_denominator = std::sqrt(denominator);
    return Eigen::Vector2d(
        options.semi_major_axis_m *
            (1.0 - options.eccentricity_squared) /
            (denominator * sqrt_denominator),
        options.semi_major_axis_m / sqrt_denominator);
}

double gravityMagnitude(
    const double latitude_rad,
    const double height_m,
    const RuntimeConfig& options) {
    if (options.gravity_model == GravityModel::kConstant) {
        return options.constant_gravity_mps2;
    }
    const double sin_latitude = std::sin(latitude_rad);
    const double sin2 = sin_latitude * sin_latitude;
    const double sin4 = sin2 * sin2;
    return 9.7803267715 *
               (1.0 + 0.0052790414 * sin2 +
                0.0000232718 * sin4) -
           (3.0877e-6 - 4.3e-9 * sin2) * height_m +
           0.72e-12 * height_m * height_m;
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

void Preintegrator::seedPreviousIncrement(const ImuIncrement& increment) {
    validateIncrement(increment);
    previous_increment_ = increment;
    has_previous_increment_ = true;
}

void Preintegrator::add(const ImuIncrement& increment) {
    validateIncrement(increment);
    validateOptions(options_);
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

NavState composeWithEnvironment(
    const NavState& initial,
    const PreintegratedDelta& delta,
    const EarthEnvironment& environment,
    const RuntimeConfig& options) {
    validateOptions(options);
    if (!environment.gravity_ned_mps2.allFinite() ||
        !environment.omega_ie_ned_radps.allFinite() ||
        !environment.omega_en_ned_radps.allFinite()) {
        throw std::invalid_argument("earth environment vectors must be finite");
    }
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
        !delta.delta_p_m.allFinite()) {
        throw std::invalid_argument("compose arguments must be finite and valid");
    }

    const Eigen::Vector3d omega_ie =
        options.enable_earth_rotation
            ? environment.omega_ie_ned_radps
            : Eigen::Vector3d::Zero();
    const Eigen::Vector3d omega_en =
        options.enable_transport_rate
            ? environment.omega_en_ned_radps
            : Eigen::Vector3d::Zero();
    Eigen::Vector3d rate_cross = omega_en;
    if (options.enable_coriolis) {
        rate_cross += 2.0 * omega_ie;
    }

    const Eigen::Quaterniond initial_attitude =
        initial.q_bn.normalized();
    const Eigen::Matrix3d c_bn = initial_attitude.toRotationMatrix();
    const Eigen::Quaterniond q_nn =
        expMap(-(omega_ie + omega_en) * delta.duration_s);
    const Eigen::Vector3d specific_force_nav =
        environment.gravity_ned_mps2 -
        rate_cross.cross(initial.velocity_ned_mps);

    NavState end = initial;
    end.time_s += delta.duration_s;
    end.q_bn =
        (q_nn * initial_attitude * delta.delta_q_b0_bk).normalized();
    end.velocity_ned_mps =
        initial.velocity_ned_mps +
        specific_force_nav * delta.duration_s +
        c_bn * delta.delta_v_mps;
    end.position_ned_m =
        initial.position_ned_m +
        initial.velocity_ned_mps * delta.duration_s +
        0.5 * specific_force_nav * delta.duration_s * delta.duration_s +
        c_bn * delta.delta_p_m;
    return end;
}

EarthEnvironment makeEarthEnvironment(
    const double latitude_rad,
    const double height_m,
    const Eigen::Vector3d& velocity_ned_mps,
    const RuntimeConfig& options) {
    validateOptions(options);
    if (!std::isfinite(latitude_rad) || !std::isfinite(height_m) ||
        !velocity_ned_mps.allFinite() ||
        latitude_rad < -kHalfPi || latitude_rad > kHalfPi) {
        throw std::invalid_argument("earth environment arguments are invalid");
    }

    const Eigen::Vector2d radii =
        meridianPrimeVerticalRadii(latitude_rad, options);
    const double rm_h = radii.x() + height_m;
    const double rn_h = radii.y() + height_m;
    const double cos_latitude = std::cos(latitude_rad);
    if (rm_h <= 0.0 || rn_h <= 0.0 ||
        std::abs(cos_latitude) <= 1e-15) {
        throw std::invalid_argument("earth environment is singular");
    }

    EarthEnvironment environment;
    environment.gravity_ned_mps2 =
        Eigen::Vector3d(
            0.0,
            0.0,
            gravityMagnitude(latitude_rad, height_m, options));
    if (options.enable_earth_rotation) {
        environment.omega_ie_ned_radps = Eigen::Vector3d(
            options.earth_rotation_rate_radps * cos_latitude,
            0.0,
            -options.earth_rotation_rate_radps *
                std::sin(latitude_rad));
    }
    if (options.enable_transport_rate) {
        environment.omega_en_ned_radps = Eigen::Vector3d(
            velocity_ned_mps.y() / rn_h,
            -velocity_ned_mps.x() / rm_h,
            -velocity_ned_mps.y() *
                std::tan(latitude_rad) / rn_h);
    }
    return environment;
}

}  // namespace preintegration_core
