#pragma once

#include "../core/constants.hpp"
#include "../core/coordinate_transform.hpp"
#include "proportional_nav.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace xsf_math {

// 导引头测量结果（Seeker Measurement Result）
struct seeker_measurement {
    bool locked = false;  // 是否锁定目标（Whether Target is Locked）
    bool in_fov = false;  // 是否在视场内（Whether Target is in Field of View）
    bool reacquired = false;  // 是否重新捕获（Whether Target is Reacquired）
    double azimuth_rad = 0.0;  // 方位角，单位弧度（Azimuth Angle in Radians）
    double elevation_rad = 0.0;  // 俯仰角，单位弧度（Elevation Angle in Radians）
    double line_of_sight_rate_rad_s = 0.0;  // 视线角速度，单位 rad/s（Line-of-Sight Rate in rad/s）
    vec3 los_unit_wcs{};  // 视线单位向量（世界坐标系）（Line-of-Sight Unit Vector in WCS）
};

// 导引头状态（Seeker State）
struct seeker_state {
    bool has_lock = false;  // 是否已锁定（Whether Lock is Established）
    double last_seen_time_s = -1.0;  // 上次见到目标的时间，单位秒（Last Seen Time in Seconds）
};

// 导引头模型（Seeker Model）
struct seeker {
    double azimuth_fov_rad = 25.0 * constants::deg_to_rad;  // 方位视场角，单位弧度（Azimuth Field of View in Radians）
    double elevation_fov_rad = 25.0 * constants::deg_to_rad;  // 俯仰视场角，单位弧度（Elevation Field of View in Radians）
    double reacquire_delay_s = 0.5;  // 重新捕获延迟，单位秒（Reacquire Delay in Seconds）
    double angle_noise_std_rad = 0.1 * constants::deg_to_rad;  // 角度噪声标准差，单位弧度（Angle Noise Standard Deviation in Radians）
    bool stochastic = false;  // 是否启用随机性（Whether Stochastic Noise is Enabled）
    mutable std::mt19937 rng{7};  // 随机数生成器（Random Number Generator）

    // 观测目标并生成测量（Observe Target and Generate Measurement）
    seeker_measurement observe(const engagement_geometry& geom,
                               const euler_angles& weapon_attitude,
                               seeker_state& state,
                               double sim_time_s) const {
        seeker_measurement out;
        vec3 los_wcs = geom.relative_pos().normalized();
        vec3 los_ecs = wcs_to_ecs(los_wcs, weapon_attitude);

        double az = std::atan2(los_ecs.y, los_ecs.x);
        double el = std::atan2(-los_ecs.z, std::max(std::sqrt(los_ecs.x * los_ecs.x + los_ecs.y * los_ecs.y), 1.0e-12));

        if (stochastic) {
            std::normal_distribution<double> noise(0.0, angle_noise_std_rad);
            az += noise(rng);
            el += noise(rng);
        }

        out.in_fov = std::abs(az) <= azimuth_fov_rad && std::abs(el) <= elevation_fov_rad;
        if (out.in_fov) {
            bool reacquired = !state.has_lock && state.last_seen_time_s >= 0.0 &&
                              (sim_time_s - state.last_seen_time_s) >= reacquire_delay_s;
            state.has_lock = true;
            state.last_seen_time_s = sim_time_s;
            out.locked = true;
            out.reacquired = reacquired;
        } else {
            state.has_lock = false;
        }

        out.azimuth_rad = az;
        out.elevation_rad = el;
        out.line_of_sight_rate_rad_s = geom.los_rate().magnitude();
        out.los_unit_wcs = los_wcs;
        return out;
    }
};

}  // namespace xsf_math
