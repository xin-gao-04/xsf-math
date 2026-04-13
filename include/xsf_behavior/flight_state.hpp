#pragma once

#include <xsf_math/core/atmosphere.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/core/vec3.hpp>
#include <cmath>

namespace xsf_math {

// 行为控制层与数学算法层分离放置。
// 这里定义的是飞行动作控制所需的轻量状态和控制命令，不负责场景推进。

// 将角度归一化到 [-pi, pi)。
// 这用于处理航向误差，避免跨越 0/360 度时出现跳变。
inline double normalize_angle_pm_pi(double angle_rad) {
    double wrapped = std::fmod(angle_rad + constants::pi, constants::two_pi);
    if (wrapped < 0.0) wrapped += constants::two_pi;
    return wrapped - constants::pi;
}

// 飞行器当前运动状态的轻量表达。
// 这一层不保存仿真对象，只表达控制器真正需要的运动学量。
struct flight_kinematic_state {
    vec3 position_wcs{};      // 世界坐标系位置
    vec3 velocity_wcs{};      // 世界坐标系速度
    vec3 acceleration_wcs{};  // 世界坐标系加速度

    double altitude_m          = 0.0;  // 高度，约定使用 -Z 为高度
    double heading_rad         = 0.0;  // 航向角
    double pitch_rad           = 0.0;  // 俯仰角
    double roll_rad            = 0.0;  // 滚转角
    double flight_path_rad     = 0.0;  // 航迹倾角
    double true_airspeed_mps   = 0.0;  // 真空速
    double ground_speed_mps    = 0.0;  // 水平速度
    double vertical_speed_mps  = 0.0;  // 升降率，向上为正
    double heading_rate_rad_s  = 0.0;  // 航向变化率
    double pitch_rate_rad_s    = 0.0;  // 俯仰角速度
    double roll_rate_rad_s     = 0.0;  // 滚转角速度
    double mach                = 0.0;  // 马赫数
    double dynamic_pressure_pa = 0.0;  // 动压

    // 用位置和速度快速构造一个控制器可消费的状态。
    // 适合外部仿真框架在没有完整气动状态对象时直接适配。
    static flight_kinematic_state from_velocity(const vec3& position_wcs,
                                                const vec3& velocity_wcs,
                                                double heading_rad = 0.0,
                                                double pitch_rad = 0.0,
                                                double roll_rad = 0.0) {
        flight_kinematic_state state;
        state.position_wcs = position_wcs;
        state.velocity_wcs = velocity_wcs;
        state.altitude_m = -position_wcs.z;
        state.true_airspeed_mps = velocity_wcs.magnitude();
        state.ground_speed_mps = std::sqrt(velocity_wcs.x * velocity_wcs.x + velocity_wcs.y * velocity_wcs.y);
        state.vertical_speed_mps = -velocity_wcs.z;
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

// 行为控制层的统一约束集合。
// 外部框架可按平台类型、飞行阶段或任务模式提供不同的约束参数。
struct flight_control_limits {
    double max_pitch_up_rad         = 30.0 * constants::deg_to_rad;  // 最大抬头角
    double max_pitch_down_rad       = 20.0 * constants::deg_to_rad;  // 最大低头角
    double max_pitch_rate_rad_s     = 10.0 * constants::deg_to_rad;  // 最大俯仰变化率
    double max_roll_angle_rad       = 60.0 * constants::deg_to_rad;  // 最大滚转角
    double max_roll_rate_rad_s      = 30.0 * constants::deg_to_rad;  // 最大滚转变化率
    double max_heading_rate_rad_s   = 6.0 * constants::deg_to_rad;   // 最大航向变化率
    double max_normal_load_factor_g = 6.0;                           // 最大法向过载
    double min_vertical_speed_mps   = -120.0;                        // 允许的最小垂向速度
    double max_vertical_speed_mps   = 120.0;                         // 允许的最大垂向速度
    double min_dynamic_pressure_pa  = 500.0;                         // 控制器生效所需最小动压
    double min_speed_mps            = 40.0;                          // 控制器生效所需最小速度
};

// 控制器输出状态码。
// `ok` 表示输出可直接消费；其余值表示控制器因为能量或目标无效而拒绝输出。
enum class flight_command_status {
    ok,
    dynamic_pressure_limited,
    speed_limited,
    invalid_target
};

// 行为控制层统一输出。
// 控制器只表达“控制意图”，具体如何驱动平台由外部框架决定。
struct flight_control_command {
    double commanded_pitch_rad            = 0.0;  // 目标俯仰
    double commanded_roll_rad             = 0.0;  // 目标滚转
    double commanded_heading_rate_rad_s   = 0.0;  // 目标航向率
    double commanded_normal_load_factor_g = 1.0;  // 目标法向过载
    double commanded_vertical_speed_mps   = 0.0;  // 目标垂向速度
    bool valid                            = true; // false 表示当前条件下不建议执行控制
    bool saturated                        = false; // true 表示命令被约束裁剪过
    flight_command_status status          = flight_command_status::ok;
};

// 检查当前速度和动压是否足以执行飞行动作控制。
inline bool flight_controls_available(const flight_kinematic_state& state,
                                      const flight_control_limits& limits) {
    return state.true_airspeed_mps >= limits.min_speed_mps &&
           state.dynamic_pressure_pa >= limits.min_dynamic_pressure_pa;
}

// 带饱和标志的通用限幅函数。
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

// 带饱和标志的角速率限制。
// 用于把目标姿态变化限制在单个时间步内可实现的范围里。
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

} // namespace xsf_math
