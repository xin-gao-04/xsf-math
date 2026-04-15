#pragma once

#include "flight_state.hpp"
#include <xsf_common/log.hpp>
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 原子控制器集合。

struct pull_up_target {
    double target_altitude_m = 0.0; // 目标高度
};

struct coordinated_turn_target {
    double target_heading_rad = 0.0; // 目标航向
};

struct descent_target {
    double target_altitude_m = 0.0; // 目标高度
};

struct level_hold_target {
    double target_altitude_m = 0.0; // 目标高度
};

struct waypoint_track_target {
    vec3 target_position_wcs{}; // 目标航点位置
    double arrive_radius_m = 50.0; // 进入该半径后认为已到达
};

struct approach_glideslope_target {
    vec3 threshold_position_wcs{};                                 // 跑道入口/下滑道参考点
    double glide_slope_rad = 3.0 * constants::deg_to_rad;         // 下滑道角
    double runway_heading_rad = 0.0;                              // 跑道方向
    double intercept_distance_m = 4000.0;                         // 参考进近距离
};

struct heading_hold_target {
    double target_heading_rad = 0.0; // 目标航向
};

struct flare_target {
    double flare_altitude_m = 15.0;                  // 开始拉平的高度门限
    double touchdown_altitude_m = 0.0;              // 接地点高度
    double target_sink_rate_mps = -1.0;             // 末段希望收敛到的下沉率，向下为负
};

namespace detail {

inline double sign_with_magnitude(double magnitude, double reference) {
    if (reference > 0.0) return magnitude;
    if (reference < 0.0) return -magnitude;
    return 0.0;
}

inline flight_control_command make_unavailable_command(const flight_kinematic_state& state,
                                                       const flight_control_limits& limits) {
    flight_control_command command;
    command.valid = false;
    command.status = (state.true_airspeed_mps < limits.min_speed_mps)
                         ? flight_command_status::speed_limited
                         : flight_command_status::dynamic_pressure_limited;
    XSF_LOG_DEBUG("flight controls unavailable: TAS={:.2f}m/s q={:.2f}Pa min_speed={:.2f}m/s min_q={:.2f}Pa",
                  state.true_airspeed_mps,
                  state.dynamic_pressure_pa,
                  limits.min_speed_mps,
                  limits.min_dynamic_pressure_pa);
    return command;
}

inline double clamp_accel(double accel_mps2, const flight_control_limits& limits, bool& saturated) {
    return clamp_with_flag(accel_mps2,
                           -max_commanded_accel_mps2(limits),
                           max_commanded_accel_mps2(limits),
                           saturated);
}

inline flight_control_command altitude_guidance_command(const flight_kinematic_state& state,
                                                        double commanded_altitude_m,
                                                        const flight_control_limits& limits,
                                                        double dt_s) {
    flight_control_command command;
    if (!flight_controls_available(state, limits)) {
        return make_unavailable_command(state, limits);
    }

    command.saturated = false;
    const double speed = std::max(state.true_airspeed_mps, 0.0);
    const double cur_fpa = state.flight_path_rad;
    const double delta_alt = commanded_altitude_m - state.altitude_m;

    double max_pitch = limits.max_pitch_up_rad;
    double min_pitch = -limits.max_pitch_down_rad;
    if (limits.max_ascent_rate_mps > 0.0 && speed > limits.max_ascent_rate_mps) {
        max_pitch = std::min(max_pitch, std::asin(limits.max_ascent_rate_mps / speed));
    }
    if (limits.max_descent_rate_mps > 0.0 && speed > limits.max_descent_rate_mps) {
        min_pitch = std::max(min_pitch, -std::asin(limits.max_descent_rate_mps / speed));
    }

    double required_vertical_speed = 0.0;
    if (delta_alt > 0.0) {
        required_vertical_speed = std::sqrt(constants::gravity_mps2 * delta_alt);
    } else if (delta_alt < 0.0) {
        required_vertical_speed = -std::sqrt(constants::gravity_mps2 * -delta_alt);
    }

    double commanded_fpa = 0.0;
    if (speed > 1.0e-8 && std::abs(required_vertical_speed) < speed) {
        commanded_fpa = std::asin(required_vertical_speed / speed);
        commanded_fpa = clamp_with_flag(commanded_fpa, min_pitch, max_pitch, command.saturated);
    } else if (delta_alt > 0.0) {
        commanded_fpa = max_pitch;
        command.saturated = true;
    } else if (delta_alt < 0.0) {
        commanded_fpa = min_pitch;
        command.saturated = true;
    }

    double delta_pitch = commanded_fpa - cur_fpa;
    double normal_accel = 0.0;
    if (dt_s > 1.0e-6 && speed > 1.0e-6 && std::abs(delta_pitch) > 1.0e-6) {
        const double path_length = speed * dt_s;
        const double radius = path_length / std::abs(delta_pitch);
        normal_accel = (speed * speed) / radius;
        if (delta_pitch < 0.0) normal_accel = -normal_accel;
    }

    // 引入重力偏置，避免高度保持时出现持续下沉。
    normal_accel += std::cos(cur_fpa) * limits.gee_bias_g * constants::gravity_mps2;
    double vertical_accel = clamp_accel(-normal_accel, limits, command.saturated);

    double commanded_pitch = clamp_with_flag(commanded_fpa,
                                             -limits.max_pitch_down_rad,
                                             limits.max_pitch_up_rad,
                                             command.saturated);
    commanded_pitch = apply_rate_limit(state.pitch_rad,
                                       commanded_pitch,
                                       limits.max_pitch_rate_rad_s,
                                       dt_s,
                                       command.saturated);

    command.commanded_pitch_rad = commanded_pitch;
    command.commanded_vertical_speed_mps =
        clamp_with_flag(speed * std::sin(commanded_fpa),
                        limits.min_vertical_speed_mps,
                        limits.max_vertical_speed_mps,
                        command.saturated);
    command.commanded_vertical_accel_mps2 = vertical_accel;
    command.commanded_normal_load_factor_g = std::abs(vertical_accel) / constants::gravity_mps2;
    XSF_LOG_TRACE("altitude guidance: alt={:.1f}m cmd_alt={:.1f}m pitch={:.3f}rad vz={:.3f}m/s az={:.3f}m/s^2 sat={}",
                  state.altitude_m,
                  commanded_altitude_m,
                  command.commanded_pitch_rad,
                  command.commanded_vertical_speed_mps,
                  command.commanded_vertical_accel_mps2,
                  command.saturated);
    return command;
}

inline flight_control_command flight_path_angle_guidance_command(const flight_kinematic_state& state,
                                                                 double commanded_fpa_rad,
                                                                 const flight_control_limits& limits,
                                                                 double dt_s) {
    constexpr double flight_path_angle_tolerance_rad = 0.0001 * constants::deg_to_rad;

    flight_control_command command;
    if (!flight_controls_available(state, limits)) {
        return make_unavailable_command(state, limits);
    }

    command.saturated = false;

    double pitch_delta = state.flight_path_rad - commanded_fpa_rad;
    pitch_delta += sign_with_magnitude(flight_path_angle_tolerance_rad, pitch_delta);

    double vertical_accel = 0.0;
    const double speed = std::max(state.true_airspeed_mps, 0.0);
    if (dt_s > 0.0 && pitch_delta != 0.0) {
        if (speed < 1.0) {
            vertical_accel = sign_with_magnitude(max_commanded_accel_mps2(limits), pitch_delta);
        } else {
            const double omega = pitch_delta / dt_s;
            const double commanded_accel = omega * speed;
            const double gravity_ecs_z = std::cos(state.flight_path_rad) * constants::gravity_mps2;
            vertical_accel = commanded_accel - gravity_ecs_z;
        }
    }
    vertical_accel = clamp_accel(vertical_accel, limits, command.saturated);

    double commanded_pitch = clamp_with_flag(commanded_fpa_rad,
                                             -limits.max_pitch_down_rad,
                                             limits.max_pitch_up_rad,
                                             command.saturated);
    commanded_pitch = apply_rate_limit(state.pitch_rad,
                                       commanded_pitch,
                                       limits.max_pitch_rate_rad_s,
                                       dt_s,
                                       command.saturated);

    command.commanded_pitch_rad = commanded_pitch;
    command.commanded_vertical_speed_mps =
        clamp_with_flag(speed * std::sin(commanded_fpa_rad),
                        limits.min_vertical_speed_mps,
                        limits.max_vertical_speed_mps,
                        command.saturated);
    command.commanded_vertical_accel_mps2 = vertical_accel;
    command.commanded_normal_load_factor_g = std::abs(vertical_accel) / constants::gravity_mps2;
    XSF_LOG_TRACE("flight-path guidance: gamma={:.4f}rad cmd_gamma={:.4f}rad pitch={:.3f}rad vz={:.3f}m/s az={:.3f}m/s^2 sat={}",
                  state.flight_path_rad,
                  commanded_fpa_rad,
                  command.commanded_pitch_rad,
                  command.commanded_vertical_speed_mps,
                  command.commanded_vertical_accel_mps2,
                  command.saturated);
    return command;
}

inline flight_control_command heading_change_command(const flight_kinematic_state& state,
                                                     double target_heading_rad,
                                                     const flight_control_limits& limits,
                                                     double dt_s) {
    flight_control_command command;
    if (!flight_controls_available(state, limits)) {
        return make_unavailable_command(state, limits);
    }

    command.saturated = false;

    const double heading_change = normalize_angle_pm_pi(target_heading_rad - state.heading_rad);
    double lateral_accel = 0.0;
    if (dt_s > 0.0) {
        if (state.ground_speed_mps > 1.0) {
            lateral_accel = (heading_change / dt_s) * state.ground_speed_mps;
        } else {
            lateral_accel = sign_with_magnitude(max_commanded_accel_mps2(limits), heading_change);
        }
    }
    lateral_accel += limits.lateral_gee_bias_g * constants::gravity_mps2;
    lateral_accel = clamp_accel(lateral_accel, limits, command.saturated);

    double commanded_heading_rate = 0.0;
    if (state.ground_speed_mps > 1.0) {
        commanded_heading_rate = lateral_accel / state.ground_speed_mps;
    }
    commanded_heading_rate = clamp_with_flag(commanded_heading_rate,
                                             -limits.max_heading_rate_rad_s,
                                             limits.max_heading_rate_rad_s,
                                             command.saturated);

    double commanded_roll = 0.0;
    if (constants::gravity_mps2 > 1.0e-8) {
        commanded_roll = std::atan2(lateral_accel, constants::gravity_mps2);
    }
    commanded_roll = clamp_with_flag(commanded_roll,
                                     -limits.max_roll_angle_rad,
                                     limits.max_roll_angle_rad,
                                     command.saturated);
    commanded_roll = apply_rate_limit(state.roll_rad,
                                      commanded_roll,
                                      limits.max_roll_rate_rad_s,
                                      dt_s,
                                      command.saturated);

    command.commanded_roll_rad = commanded_roll;
    command.commanded_heading_rate_rad_s = commanded_heading_rate;
    command.commanded_lateral_accel_mps2 = lateral_accel;
    command.commanded_normal_load_factor_g = std::abs(lateral_accel) / constants::gravity_mps2;
    command.commanded_vertical_speed_mps = state.vertical_speed_mps;
    command.commanded_pitch_rad = state.pitch_rad;
    XSF_LOG_TRACE("heading guidance: hdg={:.4f}rad cmd_hdg={:.4f}rad roll={:.3f}rad yaw_rate={:.4f}rad/s ay={:.3f}m/s^2 sat={}",
                  state.heading_rad,
                  target_heading_rad,
                  command.commanded_roll_rad,
                  command.commanded_heading_rate_rad_s,
                  command.commanded_lateral_accel_mps2,
                  command.saturated);
    return command;
}

inline flight_control_command merge_commands(const flight_control_command& lateral,
                                             const flight_control_command& vertical) {
    flight_control_command merged = vertical;
    merged.valid = lateral.valid && vertical.valid;
    merged.saturated = lateral.saturated || vertical.saturated;
    merged.status = lateral.valid ? vertical.status : lateral.status;
    merged.commanded_roll_rad = lateral.commanded_roll_rad;
    merged.commanded_heading_rate_rad_s = lateral.commanded_heading_rate_rad_s;
    merged.commanded_lateral_accel_mps2 = lateral.commanded_lateral_accel_mps2;
    merged.commanded_normal_load_factor_g =
        std::max(lateral.commanded_normal_load_factor_g, vertical.commanded_normal_load_factor_g);
    return merged;
}

} // namespace detail

struct pull_up_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const pull_up_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        return detail::altitude_guidance_command(state, target.target_altitude_m, limits, dt_s);
    }
};

struct coordinated_turn_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const coordinated_turn_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        return detail::heading_change_command(state, target.target_heading_rad, limits, dt_s);
    }
};

struct descent_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const descent_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        return detail::altitude_guidance_command(state, target.target_altitude_m, limits, dt_s);
    }
};

struct level_hold_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const level_hold_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        return detail::altitude_guidance_command(state, target.target_altitude_m, limits, dt_s);
    }
};

struct waypoint_track_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const waypoint_track_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        if (!flight_controls_available(state, limits)) {
            return detail::make_unavailable_command(state, limits);
        }

        const vec3 delta = target.target_position_wcs - state.position_wcs;
        const double horizontal_range_m = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (horizontal_range_m <= target.arrive_radius_m) {
            flight_control_command command;
            command.valid = false;
            command.status = flight_command_status::invalid_target;
            XSF_LOG_INFO("waypoint arrived: distance={:.1f}m radius={:.1f}m",
                         horizontal_range_m,
                         target.arrive_radius_m);
            return command;
        }

        return detail::heading_change_command(state, std::atan2(delta.y, delta.x), limits, dt_s);
    }
};

struct approach_glideslope_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const approach_glideslope_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        if (!flight_controls_available(state, limits)) {
            return detail::make_unavailable_command(state, limits);
        }

        const vec3 delta = target.threshold_position_wcs - state.position_wcs;
        const double horizontal_range_m = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        const double threshold_altitude_m = -target.threshold_position_wcs.z;
        const double commanded_altitude_m =
            threshold_altitude_m + std::max(horizontal_range_m, 0.0) * std::tan(target.glide_slope_rad);

        const auto lateral = detail::heading_change_command(state,
                                                            std::atan2(delta.y, delta.x),
                                                            limits,
                                                            dt_s);
        const auto vertical = detail::altitude_guidance_command(state,
                                                                commanded_altitude_m,
                                                                limits,
                                                                dt_s);
        XSF_LOG_DEBUG("approach glideslope: horiz={:.1f}m threshold_alt={:.1f}m cmd_alt={:.1f}m",
                      horizontal_range_m,
                      threshold_altitude_m,
                      commanded_altitude_m);
        return detail::merge_commands(lateral, vertical);
    }
};

struct heading_hold_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const heading_hold_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        return detail::heading_change_command(state, target.target_heading_rad, limits, dt_s);
    }
};

struct flare_controller {
    flight_control_command compute(const flight_kinematic_state& state,
                                   const flare_target& target,
                                   const flight_control_limits& limits,
                                   double dt_s) const {
        if (!flight_controls_available(state, limits)) {
            return detail::make_unavailable_command(state, limits);
        }

        const double altitude_above_touchdown_m = state.altitude_m - target.touchdown_altitude_m;
        if (altitude_above_touchdown_m > target.flare_altitude_m) {
            // 拉平窗口外维持较温和的目标下沉率。
            const double speed = std::max(state.true_airspeed_mps, 1.0);
            const double commanded_fpa = std::asin(std::clamp(target.target_sink_rate_mps / speed, -1.0, 1.0));
            return detail::flight_path_angle_guidance_command(state, commanded_fpa, limits, dt_s);
        }

        const double speed = std::max(state.true_airspeed_mps, 1.0);
        const double command_sink_rate =
            target.target_sink_rate_mps *
            std::clamp(altitude_above_touchdown_m / std::max(target.flare_altitude_m, 1.0), 0.0, 1.0);
        const double commanded_fpa = std::asin(std::clamp(command_sink_rate / speed, -1.0, 1.0));
        XSF_LOG_DEBUG("flare: alt_agl={:.2f}m sink={:.3f}m/s cmd_sink={:.3f}m/s cmd_gamma={:.4f}rad",
                      altitude_above_touchdown_m,
                      state.vertical_speed_mps,
                      command_sink_rate,
                      commanded_fpa);
        return detail::flight_path_angle_guidance_command(state, commanded_fpa, limits, dt_s);
    }
};

} // namespace xsf_math
