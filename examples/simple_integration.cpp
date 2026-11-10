#include "preintegration_core/preintegrator.hpp"

#include <cstdio>

int main() {
    preintegration_core::Preintegrator preintegrator;
    preintegration_core::ImuIncrement increment;
    increment.dt_s = 0.005;
    increment.delta_velocity_mps = Eigen::Vector3d(0.2, -0.1, 0.05) * increment.dt_s;
    increment.delta_angle_rad = Eigen::Vector3d(0.0, 0.0, 0.01) * increment.dt_s;

    for (int index = 0; index < 200; ++index) {
        preintegrator.add(increment);
    }

    preintegration_core::NavState initial;
    const Eigen::Vector3d gravity(0.0, 0.0, 9.80665);
    const preintegration_core::NavState end =
        preintegration_core::compose(
            initial, preintegrator.delta(), gravity);
    std::printf(
        "dt=%.3f s t=%.3f s dv=[%.6f, %.6f, %.6f] dp=[%.6f, %.6f, %.6f]\n",
        preintegrator.delta().duration_s,
        end.time_s,
        preintegrator.delta().delta_v_mps.x(),
        preintegrator.delta().delta_v_mps.y(),
        preintegrator.delta().delta_v_mps.z(),
        preintegrator.delta().delta_p_m.x(),
        preintegrator.delta().delta_p_m.y(),
        preintegrator.delta().delta_p_m.z());
    return 0;
}
