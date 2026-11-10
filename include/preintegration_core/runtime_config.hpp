#pragma once

namespace preintegration_core {

enum class GravityModel {
    kConstant,
    kNormalGravity,
};

// Shared experiment schema. The same field names and defaults are used by
// inertial_core so both pure-inertial paths can be configured alike.
struct RuntimeConfig {
    bool enable_earth_rotation{true};
    bool enable_transport_rate{true};
    bool enable_coriolis{true};
    bool enable_ellipsoid_radii{true};
    GravityModel gravity_model{GravityModel::kNormalGravity};

    double earth_rotation_rate_radps{7.2921151467e-5};
    double constant_gravity_mps2{9.80665};
    double semi_major_axis_m{6378137.0};
    double eccentricity_squared{0.0066943799901413156};
    double fallback_sphere_radius_m{6371000.0};

    bool enable_coning_sculling{true};
    double equal_interval_tolerance_s{1e-9};
};

}  // namespace preintegration_core
