#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "preintegration_core/runtime_config.hpp"

namespace preintegration_core {

// Bias correction and time interpolation belong to the caller. The
// preintegrator consumes bias-free increments over intervals ending at the
// measurement time.
struct ImuIncrement {
    double dt_s{0.0};
    Eigen::Vector3d delta_angle_rad{Eigen::Vector3d::Zero()};
    Eigen::Vector3d delta_velocity_mps{Eigen::Vector3d::Zero()};
};

// Relative IMU measurement between body frames b0 and bk.
// delta_q_b0_bk maps vectors expressed in bk into b0, so a body-to-navigation
// attitude composes as q_bn_k = q_bn_0 * delta_q_b0_bk. delta_v_mps and
// delta_p_m are expressed in b0.
struct PreintegratedDelta {
    double duration_s{0.0};
    Eigen::Quaterniond delta_q_b0_bk{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d delta_v_mps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d delta_p_m{Eigen::Vector3d::Zero()};
};

// Navigation state used only by the optional no-graph composition helper.
struct NavState {
    double time_s{0.0};
    Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_bn{Eigen::Quaterniond::Identity()};
};

struct EarthEnvironment {
    Eigen::Vector3d gravity_ned_mps2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_ie_ned_radps{Eigen::Vector3d::Zero()};
    Eigen::Vector3d omega_en_ned_radps{Eigen::Vector3d::Zero()};
};

using PreintegrationOptions = RuntimeConfig;

}  // namespace preintegration_core
