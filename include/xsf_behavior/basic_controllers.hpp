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

// 平飞保持动作的目标定义。
// 用于在进入目标高度附近后抑制多余的爬升/下降动作。
struct level_hold_target {
    double target_altitude_m       = 0.0;                          // 目标高度
    double altitude_gain_rad_per_m = 1.0 / 2500.0;                 // 高度误差到微小航迹角的换算增益
    double max_level_pitch_rad     = 8.0 * constants::deg_to_rad;  // 平飞保持阶段允许的最大俯仰修正
    double pitch_gain              = 1.5;                          // 俯仰误差到法向过载的增益
};

// 航点跟踪动作的目标定义。
// 当前实现仅做水平面航点跟踪，由外部框架决定何时切换到进近或终端动作。
struct waypoint_track_target {
    vec3 target_position_wcs{};                                  // 目标航点位置
    double arrive_radius_m          = 50.0;                      // 进入该半径后认为航点到达
    double heading_gain_rad_s_per_rad = 0.8;                     // 航向误差到转弯率的增益
    double max_turn_rate_rad_s      = 6.0 * constants::deg_to_rad; // 最大允许转弯率
};

// 下滑道进近动作的目标定义。
// 以跑道入口或下滑道瞄准点为参考，在水平面做对准、在垂直面做下滑道保持。
struct approach_glideslope_target {
    vec3 threshold_position_wcs{};                                 // 跑道入口或下滑道参考点
    double glide_slope_rad          = 3.0 * constants::deg_to_rad; // 下滑道角
    double altitude_gain_rad_per_m  = 1.0 / 2500.0;                // 航迹高度误差到俯仰修正的增益
    double max_descent_angle_rad    = 8.0 * constants::deg_to_rad; // 进近阶段允许的最大下滑角
    double heading_gain_rad_s_per_rad = 0.8;                       // 航向误差到转弯率的增益
    double max_turn_rate_rad_s      = 5.0 * constants::deg_to_rad; // 进近阶段最大转弯率
    double pitch_gain               = 2.0;                         // 俯仰误差到附加过载的增益
};

// 定航向保持动作的目标定义。
// 与航点跟踪不同，它直接保持一个给定航向，不关心外部几何点。
struct heading_hold_target {
    double target_heading_rad         = 0.0;                          // 目标航向
    double heading_gain_rad_s_per_rad = 0.6;                          // 航向误差到转弯率的增益
    double max_turn_rate_rad_s        = 4.0 * constants::deg_to_rad;  // 保持阶段允许的最大转弯率
};

// 拉平动作的目标定义。
// 用于进近末端在较低高度将下降率逐渐收敛到较小值。
struct flare_target {
    double flare_altitude_m            = 15.0;                        // 开始拉平的参考高度
    double touchdown_altitude_m        = 0.0;                         // 接地点高度
    double target_sink_rate_mps        = -1.0;                        // 拉平末端期望下降率
    double max_flare_pitch_up_rad      = 10.0 * constants::deg_to_rad; // 拉平阶段最大抬头角
    double flare_gain_rad_per_m        = 1.0 / 80.0;                  // 高度差到抬头修正的增益
    double sink_rate_gain_rad_per_mps  = 0.08;                        // 下降率误差到抬头修正的增益
    double pitch_gain                  = 2.0;                         // 俯仰误差到法向过载的增益
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

// 原子平飞保持控制器。
// 将高度误差转换成一个很小的俯仰修正，避免平台在目标高度附近持续大动作摆动。
struct level_hold_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const level_hold_target& target,
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

        command.saturated = false;

        // 定高保持只允许较小的俯仰修正，优先消除高度偏差和垂向速度。
        double altitude_error = target.target_altitude_m - state.altitude_m;
        double desired_flight_path = altitude_error * target.altitude_gain_rad_per_m;
        desired_flight_path = clamp_with_flag(desired_flight_path,
                                              -target.max_level_pitch_rad,
                                              target.max_level_pitch_rad,
                                              command.saturated);

        double desired_pitch = clamp_with_flag(desired_flight_path,
                                               -limits.max_pitch_down_rad,
                                               limits.max_pitch_up_rad,
                                               command.saturated);
        desired_pitch = apply_rate_limit(state.pitch_rad,
                                         desired_pitch,
                                         limits.max_pitch_rate_rad_s,
                                         dt_s,
                                         command.saturated);

        double pitch_error = desired_pitch - state.pitch_rad;
        double load_factor = 1.0 + target.pitch_gain * std::abs(pitch_error) *
                                       std::max(state.true_airspeed_mps, limits.min_speed_mps) /
                                       (2.0 * constants::gravity_mps2);
        load_factor = clamp_with_flag(load_factor, 1.0, limits.max_normal_load_factor_g, command.saturated);

        command.commanded_pitch_rad = desired_pitch;
        command.commanded_roll_rad = 0.0;
        command.commanded_heading_rate_rad_s = 0.0;
        command.commanded_vertical_speed_mps = clamp_with_flag(state.true_airspeed_mps * std::sin(desired_flight_path),
                                                               limits.min_vertical_speed_mps,
                                                               limits.max_vertical_speed_mps,
                                                               command.saturated);
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

// 原子航点跟踪控制器。
// 它把二维航点位置转换成目标航向，再复用协调转弯逻辑生成滚转和航向率指令。
struct waypoint_track_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const waypoint_track_target& target,
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

        vec3 delta = target.target_position_wcs - state.position_wcs;
        double horizontal_range_m = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (horizontal_range_m <= target.arrive_radius_m) {
            command.valid = false;
            command.status = flight_command_status::invalid_target;
            return command;
        }

        coordinated_turn_target turn_target;
        turn_target.target_heading_rad = std::atan2(delta.y, delta.x);
        turn_target.heading_gain_rad_s_per_rad = target.heading_gain_rad_s_per_rad;
        turn_target.max_turn_rate_rad_s = target.max_turn_rate_rad_s;

        coordinated_turn_controller turn_controller;
        command = turn_controller.compute(state, turn_target, limits, dt_s);
        return command;
    }
};

// 原子下滑道进近控制器。
// 横向使用航点对准，纵向使用“距阈值距离 -> 期望高度”形成基础下滑道。
struct approach_glideslope_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const approach_glideslope_target& target,
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

        vec3 delta = target.threshold_position_wcs - state.position_wcs;
        double horizontal_range_m = std::sqrt(delta.x * delta.x + delta.y * delta.y);

        command.saturated = false;

        // 横向：朝阈值点方向协调转弯。
        double desired_heading = (horizontal_range_m > 1.0e-8) ? std::atan2(delta.y, delta.x) : state.heading_rad;
        double heading_error = normalize_angle_pm_pi(desired_heading - state.heading_rad);
        double desired_heading_rate = heading_error * target.heading_gain_rad_s_per_rad;
        desired_heading_rate = clamp_with_flag(desired_heading_rate,
                                               -std::min(target.max_turn_rate_rad_s, limits.max_heading_rate_rad_s),
                                               std::min(target.max_turn_rate_rad_s, limits.max_heading_rate_rad_s),
                                               command.saturated);

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

        // 纵向：按水平距离投影出期望下滑道高度。
        double threshold_altitude_m = -target.threshold_position_wcs.z;
        double desired_altitude_m = threshold_altitude_m + horizontal_range_m * std::tan(target.glide_slope_rad);
        double altitude_error = desired_altitude_m - state.altitude_m;
        double desired_flight_path = altitude_error * target.altitude_gain_rad_per_m;
        desired_flight_path = clamp_with_flag(desired_flight_path,
                                              -target.max_descent_angle_rad,
                                              target.max_descent_angle_rad,
                                              command.saturated);

        double desired_pitch = clamp_with_flag(desired_flight_path,
                                               -limits.max_pitch_down_rad,
                                               limits.max_pitch_up_rad,
                                               command.saturated);
        desired_pitch = apply_rate_limit(state.pitch_rad,
                                         desired_pitch,
                                         limits.max_pitch_rate_rad_s,
                                         dt_s,
                                         command.saturated);

        double cos_bank = std::max(std::cos(desired_bank), 1.0e-4);
        double bank_load_factor = 1.0 / cos_bank;
        double pitch_error = std::abs(desired_pitch - state.pitch_rad);
        double pitch_load_factor = 1.0 + target.pitch_gain * pitch_error *
                                             std::max(state.true_airspeed_mps, limits.min_speed_mps) /
                                             (2.0 * constants::gravity_mps2);
        double load_factor = clamp_with_flag(std::max(bank_load_factor, pitch_load_factor),
                                             1.0,
                                             limits.max_normal_load_factor_g,
                                             command.saturated);

        command.commanded_pitch_rad = desired_pitch;
        command.commanded_roll_rad = desired_bank;
        command.commanded_heading_rate_rad_s = desired_heading_rate;
        command.commanded_vertical_speed_mps = clamp_with_flag(state.true_airspeed_mps * std::sin(desired_flight_path),
                                                               limits.min_vertical_speed_mps,
                                                               limits.max_vertical_speed_mps,
                                                               command.saturated);
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

// 原子定航向保持控制器。
// 适用于进入航向稳定段后的小幅修正，也可作为更复杂行为的横向子控制器。
struct heading_hold_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const heading_hold_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        coordinated_turn_target turn_target;
        turn_target.target_heading_rad = target.target_heading_rad;
        turn_target.heading_gain_rad_s_per_rad = target.heading_gain_rad_s_per_rad;
        turn_target.max_turn_rate_rad_s = target.max_turn_rate_rad_s;

        coordinated_turn_controller turn_controller;
        return turn_controller.compute(state, turn_target, limits, dt_s);
    }
};

// 原子拉平控制器。
// 在低高度阶段逐渐抬头并降低下沉率，为外部框架提供最后一段接地前控制意图。
struct flare_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const flare_target& target,
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

        command.saturated = false;

        // 距离接地点越近、当前下沉率越大，拉平动作应给出越明显的抬头修正。
        double altitude_above_touchdown_m = state.altitude_m - target.touchdown_altitude_m;
        double flare_window_error = std::max(target.flare_altitude_m - altitude_above_touchdown_m, 0.0);
        double sink_rate_error = target.target_sink_rate_mps - state.vertical_speed_mps;

        double desired_pitch = state.pitch_rad +
                               flare_window_error * target.flare_gain_rad_per_m -
                               sink_rate_error * target.sink_rate_gain_rad_per_mps;
        desired_pitch = clamp_with_flag(desired_pitch,
                                        -limits.max_pitch_down_rad,
                                        std::min(target.max_flare_pitch_up_rad, limits.max_pitch_up_rad),
                                        command.saturated);
        desired_pitch = apply_rate_limit(state.pitch_rad,
                                         desired_pitch,
                                         limits.max_pitch_rate_rad_s,
                                         dt_s,
                                         command.saturated);

        double pitch_error = std::max(desired_pitch - state.pitch_rad, 0.0);
        double load_factor = 1.0 + target.pitch_gain * pitch_error *
                                       std::max(state.true_airspeed_mps, limits.min_speed_mps) /
                                       constants::gravity_mps2;
        load_factor = clamp_with_flag(load_factor, 1.0, limits.max_normal_load_factor_g, command.saturated);

        command.commanded_pitch_rad = desired_pitch;
        command.commanded_roll_rad = 0.0;
        command.commanded_heading_rate_rad_s = 0.0;
        command.commanded_vertical_speed_mps = clamp_with_flag(target.target_sink_rate_mps,
                                                               limits.min_vertical_speed_mps,
                                                               limits.max_vertical_speed_mps,
                                                               command.saturated);
        command.commanded_normal_load_factor_g = load_factor;
        return command;
    }
};

} // namespace xsf_math
