#pragma once

#include "../core/constants.hpp"
#include "../core/coordinate_transform.hpp"
#include "../core/quaternion.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

struct imu_sample {
    vec3 specific_force_body_mps2{};
    vec3 gyro_body_radps{};
    double dt_s = 0.01;
};

struct gps_measurement {
    bool valid = false;
    vec3 position_wcs{};
    vec3 velocity_wcs{};
    double position_sigma_m = 5.0;
    double velocity_sigma_mps = 0.5;
};

struct gps_ins_state {
    vec3 position_wcs{};
    vec3 velocity_wcs{};
    quaternion attitude_body_to_wcs{};
    vec3 accel_bias_body_mps2{};
    vec3 gyro_bias_body_radps{};
};

struct gps_visibility {
    int satellites_visible = 0;
    double gdop = 99.0;
    bool usable = false;
};

struct gps_ins_loose_coupler {
    double position_gain = 0.2;
    double velocity_gain = 0.3;
    double bias_gain = 0.01;

    void propagate(gps_ins_state& state, const imu_sample& imu) const {
        if (imu.dt_s <= 0.0) return;

        vec3 gyro = imu.gyro_body_radps - state.gyro_bias_body_radps;
        quaternion omega_q{0.0, gyro.x, gyro.y, gyro.z};
        quaternion qdot = (state.attitude_body_to_wcs * omega_q) * 0.5;
        state.attitude_body_to_wcs = (state.attitude_body_to_wcs + qdot * imu.dt_s).normalized();

        vec3 corrected_force = imu.specific_force_body_mps2 - state.accel_bias_body_mps2;
        vec3 accel_wcs = state.attitude_body_to_wcs.rotate(corrected_force) + vec3{0.0, 0.0, constants::gravity_mps2};
        state.velocity_wcs += accel_wcs * imu.dt_s;
        state.position_wcs += state.velocity_wcs * imu.dt_s;
    }

    void update(gps_ins_state& state, const gps_measurement& gps) const {
        if (!gps.valid) return;
        vec3 pos_residual = gps.position_wcs - state.position_wcs;
        vec3 vel_residual = gps.velocity_wcs - state.velocity_wcs;
        state.position_wcs += pos_residual * position_gain;
        state.velocity_wcs += vel_residual * velocity_gain;
        state.accel_bias_body_mps2 +=
            state.attitude_body_to_wcs.inverse().rotate(vel_residual) * (bias_gain / std::max(gps.velocity_sigma_mps, 1.0));
    }
};

inline gps_visibility estimate_gps_visibility(const std::vector<vec3>& satellite_positions_ecef,
                                              const vec3& receiver_position_ecef,
                                              double min_elevation_rad = 10.0 * constants::deg_to_rad) {
    gps_visibility out;
    vec3 up = receiver_position_ecef.normalized();
    for (const auto& sat : satellite_positions_ecef) {
        vec3 los = (sat - receiver_position_ecef).normalized();
        double elevation = std::asin(std::clamp(los.dot(up), -1.0, 1.0));
        if (elevation >= min_elevation_rad) ++out.satellites_visible;
    }
    out.usable = out.satellites_visible >= 4;
    out.gdop = out.usable ? 4.0 / std::sqrt(static_cast<double>(out.satellites_visible)) : 99.0;
    return out;
}

}  // namespace xsf_math
