#pragma once

#include "flight_state.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 行为控制层的第一批原子控制器。
// 它们与 xsf-math 的公式层分开组织，便于外部框架只托管动作逻辑。

// 拉升动作的目标定义。
// 既可以直接给目标爬升角，也可以通过目标高度误差推导额外的抬头需求。
struct pull_up_target {
    double target_altitude_m       = 0.0;                           // 目标高度
    double target_climb_angle_rad  = 0.0;                           // 基础目标爬升角
    double max_climb_angle_rad     = 25.0 * constants::deg_to_rad;  // 拉升动作允许的最大爬升角
    double altitude_gain_rad_per_m = 1.0 / 1500.0;                  // 高度误差到爬升角的换算增益
    double pitch_gain              = 3.0;                           // 俯仰误差到法向过载的增益
};

// 协调转弯动作的目标定义。
struct coordinated_turn_target {
    double target_heading_rad         = 0.0;                          // 目标航向
    double heading_gain_rad_s_per_rad = 0.8;                          // 航向误差到转弯率的增益
    double max_turn_rate_rad_s        = 6.0 * constants::deg_to_rad;  // 允许的最大转弯率
};

// 下降动作的目标定义。
struct descent_target {
    double target_altitude_m        = 0.0;                           // 目标高度
    double target_descent_angle_rad = 3.0 * constants::deg_to_rad;   // 基础目标下滑角
    double max_descent_angle_rad    = 12.0 * constants::deg_to_rad;  // 最大允许下滑角
    double altitude_gain_rad_per_m  = 1.0 / 2000.0;                  // 高度误差到下滑角的换算增益
    double pitch_gain               = 2.5;                           // 俯仰变化到法向过载的增益
};

// 原子拉升控制器。
// 参考 xsf-core 中“命令姿态 + 过载限幅”的思想，输出轻量姿态/过载指令。
struct pull_up_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const pull_up_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        flight_control_command command;
        if (!flight_controls_available(state, limits)) {
            command.valid = false;
            command.status = (state.true_airspeed_mps < limits.min_speed_mps)
                                 ? flight_command_status::speed_limited
                                 : flight_command_status::dynamic_pressure_limited;
            return command;
        }

        // 目标爬升角由显式目标和高度误差两部分共同决定，取需求更大的那个。
        double altitude_error = target.target_altitude_m - state.altitude_m;
        double climb_angle_from_altitude = altitude_error * target.altitude_gain_rad_per_m;
        double desired_climb_angle = std::max(target.target_climb_angle_rad, climb_angle_from_altitude);

        command.saturated = false;
        desired_climb_angle = clamp_with_flag(desired_climb_angle,
                                              0.0,
                                              target.max_climb_angle_rad,
                                              command.saturated);

        double desired_pitch = clamp_with_flag(desired_climb_angle,
                                               -limits.max_pitch_down_rad,
                                               limits.max_pitch_up_rad,
                                               command.saturated);
        desired_pitch = apply_rate_limit(state.pitch_rad,
                                         desired_pitch,
                                         limits.max_pitch_rate_rad_s,
                                         dt_s,
                                         command.saturated);

        // 俯仰误差被转换成额外的法向过载需求。
        double pitch_error = desired_pitch - state.pitch_rad;
        double load_factor = 1.0 + target.pitch_gain * std::max(0.0, pitch_error) *
                                       std::max(state.true_airspeed_mps, limits.min_speed_mps) /
                                       constants::gravity_mps2;
        load_factor = clamp_with_flag(load_factor, 1.0, limits.max_normal_load_factor_g, command.saturated);

        command.commanded_pitch_rad = desired_pitch;
        command.commanded_roll_rad = 0.0;
        command.commanded_heading_rate_rad_s = 0.0;
        command.commanded_vertical_speed_mps = clamp_with_flag(state.true_airspeed_mps * std::sin(desired_climb_angle),
                                                               limits.min_vertical_speed_mps,
                                                               limits.max_vertical_speed_mps,
                                                               command.saturated);
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

// 原子协调转弯控制器。
// 将航向误差先换成目标航向率，再换算为协调转弯所需滚转角。
struct coordinated_turn_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const coordinated_turn_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        flight_control_command command;
        if (!flight_controls_available(state, limits)) {
            command.valid = false;
            command.status = (state.true_airspeed_mps < limits.min_speed_mps)
                                 ? flight_command_status::speed_limited
                                 : flight_command_status::dynamic_pressure_limited;
            return command;
        }

        // 先求航向误差，再按比例得到期望航向变化率。
        double heading_error = normalize_angle_pm_pi(target.target_heading_rad - state.heading_rad);
        double desired_heading_rate = heading_error * target.heading_gain_rad_s_per_rad;

        command.saturated = false;
        desired_heading_rate = clamp_with_flag(desired_heading_rate,
                                               -std::min(target.max_turn_rate_rad_s, limits.max_heading_rate_rad_s),
                                               std::min(target.max_turn_rate_rad_s, limits.max_heading_rate_rad_s),
                                               command.saturated);

        // 协调转弯近似：tan(phi) = v * psi_dot / g
        double desired_bank = 0.0;
        if (state.true_airspeed_mps > 1.0e-8) {
            desired_bank = std::atan(state.true_airspeed_mps * desired_heading_rate / constants::gravity_mps2);
        }
        desired_bank = clamp_with_flag(desired_bank,
                                       -limits.max_roll_angle_rad,
                                       limits.max_roll_angle_rad,
                                       command.saturated);
        desired_bank = apply_rate_limit(state.roll_rad,
                                        desired_bank,
                                        limits.max_roll_rate_rad_s,
                                        dt_s,
                                        command.saturated);

        double cos_bank = std::max(std::cos(desired_bank), 1.0e-4);
        double load_factor = clamp_with_flag(1.0 / cos_bank,
                                             1.0,
                                             limits.max_normal_load_factor_g,
                                             command.saturated);

        command.commanded_pitch_rad = state.pitch_rad;
        command.commanded_roll_rad = desired_bank;
        command.commanded_heading_rate_rad_s = desired_heading_rate;
        command.commanded_vertical_speed_mps = state.vertical_speed_mps;
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

// 原子下降控制器。
// 与拉升控制器对称，但目标为向下的飞行路径角和垂向速度。
struct descent_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const descent_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        flight_control_command command;
        if (!flight_controls_available(state, limits)) {
            command.valid = false;
            command.status = (state.true_airspeed_mps < limits.min_speed_mps)
                                 ? flight_command_status::speed_limited
                                 : flight_command_status::dynamic_pressure_limited;
            return command;
        }

        // 目标下滑角由显式目标和高度误差共同决定。
        double altitude_error = state.altitude_m - target.target_altitude_m;
        double descent_from_altitude = altitude_error * target.altitude_gain_rad_per_m;
        double desired_descent_angle = std::max(target.target_descent_angle_rad, descent_from_altitude);

        command.saturated = false;
        desired_descent_angle = clamp_with_flag(desired_descent_angle,
                                                0.0,
                                                target.max_descent_angle_rad,
                                                command.saturated);

        double desired_pitch = -desired_descent_angle;
        desired_pitch = clamp_with_flag(desired_pitch,
                                        -limits.max_pitch_down_rad,
                                        limits.max_pitch_up_rad,
                                        command.saturated);
        desired_pitch = apply_rate_limit(state.pitch_rad,
                                         desired_pitch,
                                         limits.max_pitch_rate_rad_s,
                                         dt_s,
                                         command.saturated);

        // 下降动作下的法向过载变化通常小于拉升，因此采用更保守的增益。
        double pitch_error = std::abs(desired_pitch - state.pitch_rad);
        double load_factor = clamp_with_flag(1.0 + target.pitch_gain * pitch_error *
                                                   std::max(state.true_airspeed_mps, limits.min_speed_mps) /
                                                   (2.0 * constants::gravity_mps2),
                                             1.0,
                                             limits.max_normal_load_factor_g,
                                             command.saturated);

        command.commanded_pitch_rad = desired_pitch;
        command.commanded_roll_rad = 0.0;
        command.commanded_heading_rate_rad_s = 0.0;
        command.commanded_vertical_speed_mps =
            clamp_with_flag(-state.true_airspeed_mps * std::sin(desired_descent_angle),
                            limits.min_vertical_speed_mps,
                            limits.max_vertical_speed_mps,
                            command.saturated);
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

} // namespace xsf_math
