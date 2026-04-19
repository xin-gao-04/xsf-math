#pragma once

#include "proportional_nav.hpp"
#include <algorithm>

namespace xsf_math {

// 预测命中点解算结果（Predicted Intercept Point Solution Result）
struct pip_solution {
    bool valid = false;  // 解是否有效（Whether the Solution is Valid）
    vec3 predicted_intercept_wcs{};  // 预测拦截点世界坐标（Predicted Intercept Point in WCS）
    double time_to_go_s = 0.0;  // 剩余飞行时间，单位秒（Time-to-Go in Seconds）
};

// 预测命中点导引（Predicted Intercept Point Guidance）
struct pip_guidance {
    double minimum_weapon_speed_mps = 50.0;  // 武器最小速度，单位 m/s（Minimum Weapon Speed in m/s）
    double lateral_gain = 2.5;  // 横向增益（Lateral Gain）
    double time_constant_s = 1.0;  // 时间常数，单位秒（Time Constant in Seconds）

    // 求解预测命中点（Solve for Predicted Intercept Point）
    pip_solution solve(const engagement_geometry& geom) const {
        pip_solution out;
        double speed = std::max(geom.weapon_vel.magnitude(), minimum_weapon_speed_mps);
        double closing = std::max(geom.closing_velocity(), speed * 0.25);
        double tgo = geom.slant_range() / std::max(closing, 1.0);
        out.predicted_intercept_wcs = geom.target_pos +
                                      geom.target_vel * tgo +
                                      0.5 * geom.target_accel * tgo * tgo;
        out.time_to_go_s = tgo;
        out.valid = true;
        return out;
    }

    // 计算加速度指令（Compute Acceleration Command）
    vec3 compute_accel(const engagement_geometry& geom) const {
        pip_solution pip = solve(geom);
        if (!pip.valid) return {};

        vec3 desired_los = (pip.predicted_intercept_wcs - geom.weapon_pos).normalized();
        double speed = std::max(geom.weapon_vel.magnitude(), minimum_weapon_speed_mps);
        vec3 desired_velocity = desired_los * speed;
        vec3 velocity_error = desired_velocity - geom.weapon_vel;
        double shaping_time = std::max(std::min(pip.time_to_go_s, time_constant_s), 0.25);
        return velocity_error * (lateral_gain / shaping_time);
    }
};

}  // namespace xsf_math
