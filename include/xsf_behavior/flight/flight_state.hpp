#pragma once

#include <xsf_math/core/atmosphere.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/core/vec3.hpp>
#include <cmath>

namespace xsf_math {

// 行为控制层使用的飞行状态和控制命令定义 (Flight state and control command definitions used by the behavior control layer).

// 将角度归一化到 [-pi, pi) (Normalize angle to [-pi, pi)).
// 这用于处理航向误差，避免跨越 0/360 度时出现跳变
// (Used for handling heading error to avoid discontinuity when crossing 0/360 degrees).
inline double normalize_angle_pm_pi(double angle_rad) {
    double wrapped = std::fmod(angle_rad + constants::pi, constants::two_pi);
    if (wrapped < 0.0) wrapped += constants::two_pi;
    return wrapped - constants::pi;
}

// 飞行器当前运动状态的轻量表达 (Lightweight representation of the current vehicle kinematic state).
struct flight_kinematic_state {
    vec3 position_wcs{};      // 世界坐标系位置 (position in World Coordinate System, WCS)
    vec3 velocity_wcs{};      // 世界坐标系速度 (velocity in WCS)
    vec3 acceleration_wcs{};  // 世界坐标系加速度 (acceleration in WCS)

    double altitude_m          = 0.0;  // 高度 (altitude)，约定使用 -Z 为高度 (convention: -Z is altitude/up)
    double heading_rad         = 0.0;  // 航向角 (heading angle)
    double pitch_rad           = 0.0;  // 俯仰角 (pitch angle)
    double roll_rad            = 0.0;  // 滚转角 (roll angle)
    double flight_path_rad     = 0.0;  // 航迹倾角 (flight path angle)
    double true_airspeed_mps   = 0.0;  // 真空速 (true airspeed, TAS)
    double ground_speed_mps    = 0.0;  // 水平速度 / 地速 (ground speed)
    double vertical_speed_mps  = 0.0;  // 升降率 (vertical speed)，向上为正 (positive upward)
    double heading_rate_rad_s  = 0.0;  // 航向变化率 (heading rate)
    double pitch_rate_rad_s    = 0.0;  // 俯仰角速度 (pitch rate)
    double roll_rate_rad_s     = 0.0;  // 滚转角速度 (roll rate)
    double mach                = 0.0;  // 马赫数 (Mach number)
    double dynamic_pressure_pa = 0.0;  // 动压 (dynamic pressure, q)

    // 用位置和速度快速构造状态 (Construct state quickly from position and velocity).
    static flight_kinematic_state from_velocity(const vec3& position_wcs,
                                                const vec3& velocity_wcs,
                                                double heading_rad = 0.0,
                                                double pitch_rad = 0.0,
                                                double roll_rad = 0.0) {
        flight_kinematic_state state;
        state.position_wcs = position_wcs;
        state.velocity_wcs = velocity_wcs;
        state.altitude_m = -position_wcs.z;  // WCS 中 Z 轴向下，故取负 (Z axis points downward in WCS, so negate)
        state.true_airspeed_mps = velocity_wcs.magnitude();
        state.ground_speed_mps = std::sqrt(velocity_wcs.x * velocity_wcs.x + velocity_wcs.y * velocity_wcs.y);
        state.vertical_speed_mps = -velocity_wcs.z;  // Z 向下，向上速度取负 (upward speed is negative of Z)
        state.heading_rad =
            (state.ground_speed_mps > 1.0e-8) ? std::atan2(velocity_wcs.y, velocity_wcs.x) : heading_rad;
        state.flight_path_rad =
            std::atan2(state.vertical_speed_mps, (state.ground_speed_mps > 1.0e-8) ? state.ground_speed_mps : 1.0);
        state.pitch_rad = pitch_rad;
        state.roll_rad = roll_rad;
        state.mach = atmosphere::mach_number(state.altitude_m, state.true_airspeed_mps);
        state.dynamic_pressure_pa = atmosphere::dynamic_pressure(state.altitude_m, state.true_airspeed_mps);
        return state;
    }
};

// 控制约束集合 (Collection of flight control limits/constraints).
struct flight_control_limits {
    double max_pitch_up_rad         = 70.0 * constants::deg_to_rad;  // 最大抬头角 (maximum pitch-up angle)
    double max_pitch_down_rad       = 70.0 * constants::deg_to_rad;  // 最大低头角 (maximum pitch-down angle)
    double max_pitch_rate_rad_s     = 10.0 * constants::deg_to_rad;  // 最大俯仰变化率 (maximum pitch rate)
    double max_roll_angle_rad       = 60.0 * constants::deg_to_rad;  // 最大滚转角 (maximum roll angle)
    double max_roll_rate_rad_s      = 30.0 * constants::deg_to_rad;  // 最大滚转变化率 (maximum roll rate)
    double max_heading_rate_rad_s   = 6.0 * constants::deg_to_rad;   // 最大航向变化率 (maximum heading rate)
    double max_normal_load_factor_g = 25.0;                          // 最大指令过载 (maximum commanded normal load factor)，单位 g
    double gee_bias_g               = 1.0;                           // 重力偏置 (gravity bias)，单位 g
    double lateral_gee_bias_g       = 0.0;                           // 横向偏置 (lateral bias)，单位 g
    double max_ascent_rate_mps      = 0.0;                           // 最大上升率 (maximum ascent rate)，0 表示不限制 (0 = unlimited)
    double max_descent_rate_mps     = 0.0;                           // 最大下降率 (maximum descent rate)，0 表示不限制 (0 = unlimited)
    double min_vertical_speed_mps   = -120.0;                        // 允许的最小垂向速度 (minimum allowed vertical speed)
    double max_vertical_speed_mps   = 120.0;                         // 允许的最大垂向速度 (maximum allowed vertical speed)
    double min_dynamic_pressure_pa  = 500.0;                         // 控制器生效所需最小动压 (minimum dynamic pressure for controls to be effective)
    double min_speed_mps            = 40.0;                          // 控制器生效所需最小速度 (minimum speed for controls to be effective)
};

// 控制器输出状态码 (Controller output status code).
// `ok` 表示输出可直接消费；其余值表示控制器因为能量或目标无效而拒绝输出
// (`ok` means output is directly consumable; other values indicate the controller refused due to energy or invalid target).
enum class flight_command_status {
    ok,
    dynamic_pressure_limited,  // 动压受限 (limited by dynamic pressure)
    speed_limited,             // 速度受限 (limited by speed)
    invalid_target             // 目标无效 (invalid target)
};

// 行为控制层统一输出 (Unified flight control command output from the behavior control layer).
struct flight_control_command {
    double commanded_pitch_rad              = 0.0;  // 目标俯仰角 (commanded pitch angle)
    double commanded_roll_rad               = 0.0;  // 目标滚转角 (commanded roll angle)
    double commanded_heading_rate_rad_s     = 0.0;  // 目标航向率 (commanded heading rate)
    double commanded_normal_load_factor_g   = 0.0;  // 等效法向加速度折算成 g (equivalent normal load factor in g)
    double commanded_vertical_speed_mps     = 0.0;  // 目标垂向速度 (commanded vertical speed)
    double commanded_lateral_accel_mps2     = 0.0;  // ECS Y 指令加速度 (lateral acceleration command in ECS Y axis)
    double commanded_vertical_accel_mps2    = 0.0;  // ECS Z 指令加速度 (vertical acceleration command in ECS Z axis)
    bool valid                              = true;  // false 表示当前条件下不执行控制 (false = do not execute control under current conditions)
    bool saturated                          = false; // true 表示命令被约束裁剪过 (true = command was clipped by constraints)
    flight_command_status status            = flight_command_status::ok;  // 当前状态 (current status)
};

// 检查当前速度和动压是否足以执行飞行动作控制
// (Check whether current speed and dynamic pressure are sufficient for flight control actuation).
inline bool flight_controls_available(const flight_kinematic_state& state,
                                      const flight_control_limits& limits) {
    return state.true_airspeed_mps >= limits.min_speed_mps &&
           state.dynamic_pressure_pa >= limits.min_dynamic_pressure_pa;
}

// 带饱和标志的通用限幅函数 (Generic clamp function with saturation flag).
inline double clamp_with_flag(double value, double lo, double hi, bool& saturated) {
    if (value < lo) {
        saturated = true;
        return lo;
    }
    if (value > hi) {
        saturated = true;
        return hi;
    }
    return value;
}

// 带饱和标志的角速率限制 (Rate limit with saturation flag).
// 用于把目标姿态变化限制在单个时间步内可实现的范围里
// (Used to constrain attitude change to achievable range within a single time step).
inline double apply_rate_limit(double current_value,
                               double target_value,
                               double max_rate_rad_s,
                               double dt_s,
                               bool& saturated) {
    if (dt_s <= 0.0 || max_rate_rad_s <= 0.0) return target_value;

    double delta = target_value - current_value;
    double limit = max_rate_rad_s * dt_s;
    if (delta > limit) {
        saturated = true;
        return current_value + limit;
    }
    if (delta < -limit) {
        saturated = true;
        return current_value - limit;
    }
    return target_value;
}

// 返回当前限制下可用的最大指令加速度 (Return maximum commanded acceleration available under current limits).
inline double max_commanded_accel_mps2(const flight_control_limits& limits) {
    return std::max(limits.max_normal_load_factor_g, 0.0) * constants::gravity_mps2;
}

} // namespace xsf_math
