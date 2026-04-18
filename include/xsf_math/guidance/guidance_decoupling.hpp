#pragma once

#include "../core/constants.hpp"
#include "../core/coordinate_transform.hpp"
#include "../core/vec3.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct guidance_channel_command {
    double normal_accel_mps2 = 0.0;    // 俯仰法向
    double lateral_accel_mps2 = 0.0;   // 偏航侧向
    double normal_load_g = 0.0;
    double lateral_load_g = 0.0;
    double pitch_cmd_rad = 0.0;
    double yaw_cmd_rad = 0.0;
};

inline guidance_channel_command decompose_guidance_accel(const vec3& accel_wcs,
                                                         const euler_angles& attitude,
                                                         double speed_mps,
                                                         double response_time_s = 1.0) {
    vec3 accel_ecs = wcs_to_ecs(accel_wcs, attitude);
    guidance_channel_command cmd;
    cmd.normal_accel_mps2 = -accel_ecs.z;
    cmd.lateral_accel_mps2 = accel_ecs.y;
    cmd.normal_load_g = cmd.normal_accel_mps2 / constants::gravity_mps2;
    cmd.lateral_load_g = cmd.lateral_accel_mps2 / constants::gravity_mps2;

    double effective_speed = std::max(speed_mps, 1.0);
    double tau = std::max(response_time_s, 0.05);
    cmd.pitch_cmd_rad = std::atan2(cmd.normal_accel_mps2 * tau, effective_speed);
    cmd.yaw_cmd_rad = std::atan2(cmd.lateral_accel_mps2 * tau, effective_speed);
    return cmd;
}

inline vec3 guidance_channels_to_accel_wcs(const guidance_channel_command& cmd,
                                           const euler_angles& attitude) {
    vec3 accel_ecs{0.0, cmd.lateral_accel_mps2, -cmd.normal_accel_mps2};
    return ecs_to_wcs(accel_ecs, attitude);
}

}  // namespace xsf_math
