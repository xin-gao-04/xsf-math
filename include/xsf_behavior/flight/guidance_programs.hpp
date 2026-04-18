#pragma once

#include "flight_state.hpp"
#include <xsf_common/log.hpp>
#include <xsf_math/core/coordinate_transform.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>
#include <xsf_math/orbital/kepler.hpp>
#include <algorithm>
#include <cmath>
#include <optional>

namespace xsf_math {

// 程序级制导逻辑集合。

enum class guidance_program_status {
    continue_running,
    complete
};

enum class guidance_pn_method {
    pure,
    augmented
};

struct guidance_program_commands {
    vec3 accel_cmd_ecs{};           // ECS 加速度命令
    vec3 angle_rate_cmd_rad_s{};    // 姿态角速率命令，顺序为 roll/pitch/yaw
    bool saturated = false;         // 是否被相位限制裁剪
};

struct guidance_program_result {
    guidance_program_commands commands{};
    guidance_program_status status = guidance_program_status::continue_running;
};

struct guidance_phase_options {
    double cos_los_offset = 0.866;
    double pn_gain = 3.0;
    double vp_gain = 10.0;
    double gee_bias = 1.0;
    double lateral_gee_bias = 0.0;
    double max_gee_cmd_mps2 = 25.0 * constants::gravity_mps2;
    double max_pitch_angle_rad = 70.0 * constants::deg_to_rad;
    double max_ascent_rate_mps = 0.0;
    double max_descent_rate_mps = 0.0;
    double time_constant_s = 0.0;
    guidance_pn_method pn_method = guidance_pn_method::pure;
    std::optional<double> commanded_altitude_m;
    bool commanded_altitude_is_agl = false;
    std::optional<double> commanded_flight_path_angle_rad;
    std::optional<double> commanded_azimuth_offset_rad;
};

struct guidance_program_state {
    flight_kinematic_state vehicle{};
    euler_angles commanded_attitude{}; // 当前命令姿态
    bool commanded_attitude_valid = false;
    double sim_time_s = 0.0;
    double current_time_s = 0.0;
    double end_time_s = 0.0;
    double last_update_time_s = 0.0;

    bool aimpoint_is_valid = false;
    vec3 aimpoint_position_wcs{};
    vec3 target_velocity_wcs{};
    vec3 target_acceleration_wcs{};

    bool terrain_enabled = false;
    double terrain_height_m = 0.0;

    engagement_geometry engagement() const {
        engagement_geometry geom;
        geom.weapon_pos = vehicle.position_wcs;
        geom.weapon_vel = vehicle.velocity_wcs;
        geom.target_pos = aimpoint_position_wcs;
        geom.target_vel = target_velocity_wcs;
        geom.target_accel = target_acceleration_wcs;
        return geom;
    }

    vec3 aim_rel_loc_wcs() const {
        return aimpoint_position_wcs - vehicle.position_wcs;
    }

    euler_angles attitude() const {
        return {vehicle.heading_rad, vehicle.pitch_rad, vehicle.roll_rad};
    }

    euler_angles current_commanded_attitude() const {
        if (commanded_attitude_valid) return commanded_attitude;
        return attitude();
    }

    vec3 aim_unit_vec_ecs() const {
        return wcs_to_ecs(aim_rel_loc_wcs().normalized(), attitude());
    }

    double step_dt_s() const {
        return end_time_s - current_time_s;
    }

    double update_dt_s() const {
        return current_time_s - last_update_time_s;
    }
};

struct attitude_axis_target {
    std::optional<double> angle_rad;
    std::optional<double> rate_rad_s;
    bool body_angle = true;
};

namespace detail {

inline double sign_of(double magnitude, double reference) {
    if (reference > 0.0) return magnitude;
    if (reference < 0.0) return -magnitude;
    return 0.0;
}

inline double normalize_axis_angle(int axis, double angle_rad) {
    if (axis == 1) {
        return std::clamp(angle_rad, -constants::half_pi, constants::half_pi);
    }
    return normalize_angle_pm_pi(angle_rad);
}

inline double clamp_program_accel(double accel_mps2, const guidance_phase_options& phase, bool& saturated) {
    return clamp_with_flag(accel_mps2, -phase.max_gee_cmd_mps2, phase.max_gee_cmd_mps2, saturated);
}

inline void limit_program_accel(guidance_program_commands& commands, const guidance_phase_options& phase) {
    commands.accel_cmd_ecs.y = clamp_with_flag(commands.accel_cmd_ecs.y,
                                               -phase.max_gee_cmd_mps2,
                                               phase.max_gee_cmd_mps2,
                                               commands.saturated);
    commands.accel_cmd_ecs.z = clamp_with_flag(commands.accel_cmd_ecs.z,
                                               -phase.max_gee_cmd_mps2,
                                               phase.max_gee_cmd_mps2,
                                               commands.saturated);
}

inline void apply_gravity_bias(const guidance_program_state& state,
                               double gravity_bias_factor,
                               guidance_program_commands& commands) {
    if (gravity_bias_factor == 0.0) return;
    const vec3 gravity_wcs{0.0, 0.0, constants::gravity_mps2};
    const vec3 gravity_ecs = wcs_to_ecs(gravity_wcs, state.attitude());
    commands.accel_cmd_ecs += (-gravity_bias_factor) * gravity_ecs;
}

inline void apply_lateral_bias(double lateral_bias_factor, guidance_program_commands& commands) {
    if (lateral_bias_factor == 0.0) return;
    commands.accel_cmd_ecs.y += lateral_bias_factor * constants::gravity_mps2;
}

inline void altitude_guidance(const guidance_program_state& state,
                              const guidance_phase_options& phase,
                              guidance_program_commands& commands,
                              double commanded_altitude_m) {
    const double delta_t = state.update_dt_s();
    if (delta_t < 1.0e-6) return;

    double effective_commanded_altitude_m = commanded_altitude_m;
    if (phase.commanded_altitude_is_agl && state.terrain_enabled) {
        effective_commanded_altitude_m += state.terrain_height_m;
    }

    const double cur_alt = state.vehicle.altitude_m;
    const double cur_pitch = state.vehicle.flight_path_rad;
    const double cur_speed = state.vehicle.true_airspeed_mps;
    const double delta_alt = effective_commanded_altitude_m - cur_alt;

    double max_pitch_angle = phase.max_pitch_angle_rad;
    double min_pitch_angle = -phase.max_pitch_angle_rad;
    if (phase.max_ascent_rate_mps > 0.0 && cur_speed > phase.max_ascent_rate_mps) {
        max_pitch_angle = std::min(max_pitch_angle, std::asin(phase.max_ascent_rate_mps / cur_speed));
    }
    if (phase.max_descent_rate_mps > 0.0 && cur_speed > phase.max_descent_rate_mps) {
        min_pitch_angle = std::max(min_pitch_angle, -std::asin(phase.max_descent_rate_mps / cur_speed));
    }

    double required_vertical_speed = 0.0;
    if (delta_alt >= 0.0) {
        required_vertical_speed = std::sqrt(constants::gravity_mps2 * delta_alt);
    } else {
        required_vertical_speed = -std::sqrt(constants::gravity_mps2 * -delta_alt);
    }

    double new_pitch = 0.0;
    if (std::abs(required_vertical_speed) < cur_speed && cur_speed > 1.0e-8) {
        new_pitch = std::asin(required_vertical_speed / cur_speed);
        new_pitch = std::clamp(new_pitch, min_pitch_angle, max_pitch_angle);
    } else if (delta_alt >= 0.0) {
        new_pitch = max_pitch_angle;
    } else {
        new_pitch = min_pitch_angle;
    }

    const double delta_pitch = new_pitch - cur_pitch;
    double normal_accel = 0.0;
    if (std::abs(delta_pitch) > 1.0e-6 && cur_speed > 1.0e-6) {
        const double path_length = cur_speed * delta_t;
        const double radius = path_length / std::abs(delta_pitch);
        normal_accel = (cur_speed * cur_speed) / radius;
        if (delta_pitch < 0.0) normal_accel = -normal_accel;
    }

    normal_accel += std::cos(state.vehicle.flight_path_rad) * constants::gravity_mps2;
    commands.accel_cmd_ecs.z = -normal_accel;
}

inline bool flight_path_angle_achieved(double current_fpa_rad,
                                       double last_fpa_rad,
                                       double commanded_fpa_rad) {
    constexpr double tolerance_rad = 0.0001 * constants::deg_to_rad;
    return (std::abs(current_fpa_rad - commanded_fpa_rad) <= tolerance_rad) ||
           ((current_fpa_rad <= commanded_fpa_rad) && (last_fpa_rad >= commanded_fpa_rad)) ||
           ((current_fpa_rad >= commanded_fpa_rad) && (last_fpa_rad <= commanded_fpa_rad));
}

inline void flight_path_angle_guidance(const guidance_program_state& state,
                                       const guidance_phase_options& phase,
                                       double commanded_fpa_rad,
                                       guidance_program_commands& commands) {
    constexpr double tolerance_rad = 0.0001 * constants::deg_to_rad;

    double pitch_delta = state.vehicle.flight_path_rad - commanded_fpa_rad;
    pitch_delta += sign_of(tolerance_rad, pitch_delta);

    const double dt_s = state.step_dt_s();
    if (dt_s <= 0.0 || pitch_delta == 0.0) return;

    const double speed = state.vehicle.true_airspeed_mps;
    if (speed < 1.0) {
        commands.accel_cmd_ecs.y = 0.0;
        commands.accel_cmd_ecs.z = sign_of(phase.max_gee_cmd_mps2, pitch_delta);
        return;
    }

    const double omega = pitch_delta / dt_s;
    const double commanded_accel = omega * speed;
    const vec3 gravity_wcs{0.0, 0.0, constants::gravity_mps2};
    const vec3 gravity_ecs = wcs_to_ecs(gravity_wcs, state.attitude());
    const vec3 cmd_accel_ecs{0.0, 0.0, commanded_accel};
    const vec3 compensated = cmd_accel_ecs - gravity_ecs;
    commands.accel_cmd_ecs.z = compensated.z;
}

inline void pro_nav_guidance(const guidance_program_state& state,
                             double pro_nav_gain,
                             bool augmented,
                             guidance_program_commands& commands) {
    const engagement_geometry geom = state.engagement();
    const double rel_range = geom.slant_range();
    if (rel_range < 1.0) {
        commands.accel_cmd_ecs.y = 0.0;
        commands.accel_cmd_ecs.z = 0.0;
        return;
    }

    vec3 accel_wcs = geom.los_rate().cross(geom.weapon_vel);
    if (augmented) {
        accel_wcs += -0.5 * geom.target_accel;
    }
    accel_wcs *= pro_nav_gain;

    const vec3 accel_ecs = wcs_to_ecs(accel_wcs, state.attitude());
    commands.accel_cmd_ecs.y = accel_ecs.y;
    commands.accel_cmd_ecs.z = accel_ecs.z;
}

inline void pursuit_guidance(const guidance_program_state& state,
                             double pursuit_gain,
                             guidance_program_commands& commands) {
    const vec3 rel = state.aim_rel_loc_wcs();
    if (rel.magnitude() < 1.0) {
        commands.accel_cmd_ecs.y = 0.0;
        commands.accel_cmd_ecs.z = 0.0;
        return;
    }

    const vec3 weapon_unit_vel_ecs{1.0, 0.0, 0.0};
    const vec3 aim_unit_ecs = state.aim_unit_vec_ecs();
    const vec3 z_axis_ecs = weapon_unit_vel_ecs.cross(aim_unit_ecs);
    const vec3 pursuit_vec_ecs = z_axis_ecs.cross(weapon_unit_vel_ecs);
    const double pursuit_mag = pursuit_vec_ecs.magnitude();
    if (pursuit_mag < 1.0e-12) {
        commands.accel_cmd_ecs.y = 0.0;
        commands.accel_cmd_ecs.z = 0.0;
        return;
    }

    const vec3 unit = pursuit_vec_ecs / pursuit_mag;
    commands.accel_cmd_ecs.y = pursuit_mag * pursuit_gain * constants::gravity_mps2 * unit.y;
    commands.accel_cmd_ecs.z = pursuit_mag * pursuit_gain * constants::gravity_mps2 * unit.z;
}

inline void angle_offset_guidance(const guidance_program_state& state,
                                  const guidance_phase_options& phase,
                                  guidance_program_commands& commands) {
    if (!phase.commanded_azimuth_offset_rad.has_value()) return;
    const double dt_s = state.step_dt_s();
    if (dt_s <= 0.0) return;

    const vec3 tgt_loc = state.aim_rel_loc_wcs();
    const double target_bearing = std::atan2(tgt_loc.y, tgt_loc.x);
    const double relative_target_bearing = normalize_angle_pm_pi(target_bearing - state.vehicle.heading_rad);
    const double commanded_offset = *phase.commanded_azimuth_offset_rad;

    double heading_change = 0.0;
    if (relative_target_bearing >= 0.0) {
        heading_change = -(commanded_offset - relative_target_bearing);
    } else {
        heading_change = -((-commanded_offset) - relative_target_bearing);
    }

    if (state.vehicle.ground_speed_mps > 1.0) {
        commands.accel_cmd_ecs.y = (heading_change / dt_s) * state.vehicle.ground_speed_mps;
    } else {
        commands.accel_cmd_ecs.y = sign_of(phase.max_gee_cmd_mps2, heading_change);
    }
}

} // namespace detail

class legacy_guidance_program {
public:
    guidance_program_result compute(const guidance_program_state& state,
                                    const guidance_phase_options& phase) {
        guidance_program_result result;
        bool apply_gravity_bias = true;

        double y_accel = 0.0;
        double z_accel = 0.0;

        if (state.aimpoint_is_valid) {
            bool use_pursuit = phase.vp_gain > 0.0;
            if (phase.pn_gain > 0.0 && phase.vp_gain > 0.0 && state.aim_unit_vec_ecs().x > phase.cos_los_offset) {
                use_pursuit = false;
            }

            guidance_program_commands los_commands;
            if (use_pursuit) {
                detail::pursuit_guidance(state, phase.vp_gain, los_commands);
            } else if (phase.pn_gain > 0.0) {
                detail::pro_nav_guidance(state,
                                         phase.pn_gain,
                                         phase.pn_method == guidance_pn_method::augmented,
                                         los_commands);
            }

            detail::angle_offset_guidance(state, phase, los_commands);
            y_accel = los_commands.accel_cmd_ecs.y;
            z_accel = los_commands.accel_cmd_ecs.z;

            if (phase.time_constant_s > 0.0) {
                const double dt_s = state.step_dt_s();
                if (dt_s > 0.0) {
                    const double factor = std::exp(-dt_s / phase.time_constant_s);
                    y_accel = (last_y_accel_mps2_ * factor) + ((1.0 - factor) * y_accel);
                    z_accel = (last_z_accel_mps2_ * factor) + ((1.0 - factor) * z_accel);
                }
            }
        }

        last_y_accel_mps2_ = y_accel;
        last_z_accel_mps2_ = z_accel;
        result.commands.accel_cmd_ecs.y = y_accel;
        result.commands.accel_cmd_ecs.z = z_accel;

        if (phase.commanded_flight_path_angle_rad.has_value()) {
            detail::flight_path_angle_guidance(state, phase, *phase.commanded_flight_path_angle_rad, result.commands);
            apply_gravity_bias = false;
        }

        if (phase.commanded_altitude_m.has_value()) {
            detail::altitude_guidance(state, phase, result.commands, *phase.commanded_altitude_m);
            apply_gravity_bias = false;
        }

        if (apply_gravity_bias) {
            detail::apply_gravity_bias(state, phase.gee_bias, result.commands);
        }

        detail::apply_lateral_bias(phase.lateral_gee_bias, result.commands);
        detail::limit_program_accel(result.commands, phase);
        XSF_LOG_DEBUG("legacy guidance: aim_valid={} ay={:.3f} az={:.3f} sat={}",
                      state.aimpoint_is_valid,
                      result.commands.accel_cmd_ecs.y,
                      result.commands.accel_cmd_ecs.z,
                      result.commands.saturated);
        return result;
    }

private:
    double last_y_accel_mps2_ = 0.0;
    double last_z_accel_mps2_ = 0.0;
};

class altitude_guidance_program {
public:
    std::optional<double> commanded_altitude_m;
    bool commanded_altitude_is_agl = false;

    guidance_program_result compute(const guidance_program_state& state,
                                    guidance_phase_options phase) const {
        guidance_program_result result;
        if (commanded_altitude_m.has_value()) {
            phase.commanded_altitude_m = commanded_altitude_m;
            phase.commanded_altitude_is_agl = commanded_altitude_is_agl;
        }
        if (phase.commanded_altitude_m.has_value()) {
            detail::altitude_guidance(state, phase, result.commands, *phase.commanded_altitude_m);
        }
        detail::limit_program_accel(result.commands, phase);
        XSF_LOG_DEBUG("altitude program: cmd_alt={} ay={:.3f} az={:.3f} sat={}",
                      phase.commanded_altitude_m.value_or(0.0),
                      result.commands.accel_cmd_ecs.y,
                      result.commands.accel_cmd_ecs.z,
                      result.commands.saturated);
        return result;
    }
};

class intercept_guidance_program {
public:
    std::optional<double> pro_nav_gain;
    std::optional<double> cos_switch_angle;
    std::optional<double> pursuit_nav_gain;
    std::optional<guidance_pn_method> pn_method;

    guidance_program_result compute(const guidance_program_state& state,
                                    const guidance_phase_options& phase) const {
        guidance_program_result result;
        if (!state.aimpoint_is_valid) {
            return result;
        }

        const double pursuit_gain = pursuit_nav_gain.value_or(phase.vp_gain);
        const double pn_gain_value = pro_nav_gain.value_or(phase.pn_gain);
        const double switch_angle = cos_switch_angle.value_or(phase.cos_los_offset);
        const guidance_pn_method selected_method = pn_method.value_or(phase.pn_method);

        bool use_pursuit = pursuit_gain > 0.0;
        if ((pn_gain_value > 0.0) && (pursuit_gain > 0.0) && (state.aim_unit_vec_ecs().x > switch_angle)) {
            use_pursuit = false;
        }

        if (use_pursuit) {
            detail::pursuit_guidance(state, pursuit_gain, result.commands);
        } else if (pn_gain_value > 0.0) {
            detail::pro_nav_guidance(state,
                                     pn_gain_value,
                                     selected_method == guidance_pn_method::augmented,
                                     result.commands);
        }

        detail::limit_program_accel(result.commands, phase);
        XSF_LOG_DEBUG("intercept program: pursuit={} ay={:.3f} az={:.3f} sat={}",
                      use_pursuit,
                      result.commands.accel_cmd_ecs.y,
                      result.commands.accel_cmd_ecs.z,
                      result.commands.saturated);
        return result;
    }
};

class legacy_flight_path_angle_program {
public:
    std::optional<double> commanded_flight_path_angle_rad;

    guidance_program_result compute(const guidance_program_state& state,
                                    const guidance_phase_options& phase) {
        guidance_program_result result;
        std::optional<double> command_angle = commanded_flight_path_angle_rad;
        if (!command_angle.has_value()) {
            command_angle = phase.commanded_flight_path_angle_rad;
        }

        if (!command_angle.has_value()) {
            result.status = guidance_program_status::complete;
            return result;
        }

        if (detail::flight_path_angle_achieved(state.vehicle.flight_path_rad, last_flight_path_angle_rad_, *command_angle)) {
            result.status = guidance_program_status::complete;
        } else {
            detail::flight_path_angle_guidance(state, phase, *command_angle, result.commands);
            detail::limit_program_accel(result.commands, phase);
        }

        last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
        XSF_LOG_DEBUG("legacy FPA program: cmd_gamma={:.4f} current_gamma={:.4f} status={}",
                      *command_angle,
                      state.vehicle.flight_path_rad,
                      result.status == guidance_program_status::complete ? "complete" : "running");
        return result;
    }

private:
    double last_flight_path_angle_rad_ = 0.0;
};

class gravity_bias_program {
public:
    std::optional<double> gravity_bias_factor;

    guidance_program_result compute(const guidance_program_state& state,
                                    const guidance_phase_options& phase) const {
        guidance_program_result result;
        detail::apply_gravity_bias(state, gravity_bias_factor.value_or(phase.gee_bias), result.commands);
        detail::limit_program_accel(result.commands, phase);
        XSF_LOG_TRACE("gravity bias program: bias={:.3f} ay={:.3f} az={:.3f}",
                      gravity_bias_factor.value_or(phase.gee_bias),
                      result.commands.accel_cmd_ecs.y,
                      result.commands.accel_cmd_ecs.z);
        return result;
    }
};

class gravity_turn_program {
public:
    guidance_program_result compute(const guidance_program_state&,
                                    const guidance_phase_options&) const {
        guidance_program_result result;
        result.commands.accel_cmd_ecs.z = 0.0;
        return result;
    }
};

class attitude_guidance_program {
public:
    attitude_axis_target yaw{};
    attitude_axis_target pitch{};
    attitude_axis_target roll{};
    double default_angle_rate_rad_s = 10.0 * constants::deg_to_rad;

    guidance_program_result compute(const guidance_program_state& state) const {
        guidance_program_result result;
        const double dt_s = state.step_dt_s();
        if (dt_s <= 0.0) {
            result.status = guidance_program_status::complete;
            return result;
        }

        const euler_angles current_commanded = state.current_commanded_attitude();
        const euler_angles current_body = state.attitude();
        const double current_angle[3] = {current_commanded.roll_rad, current_commanded.pitch_rad, current_commanded.heading_rad};
        const double base_angle[3] = {current_body.roll_rad, current_body.pitch_rad, current_body.heading_rad};

        const attitude_axis_target* targets[3] = {&roll, &pitch, &yaw};
        bool continuous_update_needed = false;
        int angles_tested = 0;
        int angles_completed = 0;
        constexpr double completion_tolerance_rad = 0.01 * constants::deg_to_rad;

        for (int axis = 0; axis < 3; ++axis) {
            const auto& target = *targets[axis];
            if (target.angle_rad.has_value()) {
                double requested_angle = *target.angle_rad;
                if (!target.body_angle) {
                    requested_angle -= base_angle[axis];
                    requested_angle = detail::normalize_axis_angle(axis, requested_angle);
                }

                const double angle_to_go =
                    detail::normalize_axis_angle(axis, requested_angle - current_angle[axis]);
                const double configured_rate = target.rate_rad_s.value_or(default_angle_rate_rad_s);
                const double commanded_rate = std::min(std::abs(angle_to_go) / dt_s, configured_rate);
                result.commands.angle_rate_cmd_rad_s[axis] = detail::sign_of(commanded_rate, angle_to_go);
                ++angles_tested;
                if (std::abs(angle_to_go) <= completion_tolerance_rad) {
                    ++angles_completed;
                }
            } else if (target.rate_rad_s.has_value()) {
                result.commands.angle_rate_cmd_rad_s[axis] = *target.rate_rad_s;
                continuous_update_needed = true;
            }
        }

        if (!continuous_update_needed && angles_completed == angles_tested) {
            result.status = guidance_program_status::complete;
        }
        XSF_LOG_DEBUG("attitude program: roll_rate={:.4f} pitch_rate={:.4f} yaw_rate={:.4f} status={}",
                      result.commands.angle_rate_cmd_rad_s.x,
                      result.commands.angle_rate_cmd_rad_s.y,
                      result.commands.angle_rate_cmd_rad_s.z,
                      result.status == guidance_program_status::complete ? "complete" : "running");
        return result;
    }
};

class flight_path_angle_guidance_program {
public:
    std::optional<double> commanded_flight_path_angle_rad;
    double pitch_rate_rad_s = 0.15 * constants::deg_to_rad;
    double time_constant_s = 1.0;

    guidance_program_result compute(const guidance_program_state& state,
                                    const guidance_phase_options& phase) {
        guidance_program_result result;

        std::optional<double> command_angle = commanded_flight_path_angle_rad;
        if (!command_angle.has_value()) {
            command_angle = phase.commanded_flight_path_angle_rad;
        }
        if (!command_angle.has_value()) {
            result.status = guidance_program_status::complete;
            return result;
        }

        double commanded_fpa = *command_angle;
        commanded_fpa = std::clamp(commanded_fpa, -0.99 * constants::half_pi, 0.99 * constants::half_pi);
        double pitch_delta = commanded_fpa - state.vehicle.flight_path_rad;
        constexpr double tolerance_rad = 0.0001 * constants::deg_to_rad;
        pitch_delta += detail::sign_of(tolerance_rad, pitch_delta);

        if (first_execute_call_) {
            start_time_s_ = state.current_time_s;
            start_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
            last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
            pitch_down_program_ = (pitch_delta <= 0.0);
            switch_time_s_ = -1.0;
            first_execute_call_ = false;
        }

        const vec3 own_loc = state.vehicle.position_wcs;
        const double radius_sq = std::max(own_loc.magnitude_sq(), 1.0);
        const double gravity_mps2 = mu_earth / radius_sq;
        const double grav_accel_z = gravity_mps2 * std::cos(state.vehicle.flight_path_rad);
        const double speed = std::max(state.vehicle.true_airspeed_mps, 1.0);

        double cmd_accel_z = 0.0;
        double pitch_accel_z = 0.0;
        double pitch_rate_cmd = 0.0;

        if (time_constant_s != 0.0) {
            if (pitch_down_program_) {
                if (detail::flight_path_angle_achieved(state.vehicle.flight_path_rad,
                                                       last_flight_path_angle_rad_,
                                                       commanded_fpa)) {
                    result.status = guidance_program_status::complete;
                    last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
                    return result;
                }

                if (switch_time_s_ < 0.0) {
                    double factor = 1.0;
                    const double elapsed = state.end_time_s - start_time_s_;
                    if (elapsed < 10.0 * time_constant_s) {
                        factor = std::max(1.0 - std::exp(-elapsed / time_constant_s), 0.01);
                    }
                    pitch_accel_z =
                        factor * (pitch_rate_rad_s * speed) * detail::sign_of(1.0, -pitch_delta);
                    cmd_accel_z = pitch_accel_z - grav_accel_z;

                    const double denom = commanded_fpa - start_flight_path_angle_rad_;
                    double pitch_fraction = 0.0;
                    if (std::abs(denom) > 1.0e-9) {
                        pitch_fraction = (state.vehicle.flight_path_rad - start_flight_path_angle_rad_) / denom;
                    }
                    if (factor >= 0.98 || pitch_fraction >= 0.5) {
                        switch_time_s_ = state.current_time_s;
                        switch_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
                        switch_lateral_acceleration_mps2_ = cmd_accel_z;
                    }
                } else {
                    const double denom = commanded_fpa - switch_flight_path_angle_rad_;
                    double factor = 0.0;
                    if (std::abs(denom) > 1.0e-9) {
                        factor = 1.0 - ((state.vehicle.flight_path_rad - switch_flight_path_angle_rad_) / denom);
                    }
                    cmd_accel_z = factor * switch_lateral_acceleration_mps2_;
                }

                pitch_accel_z = cmd_accel_z + grav_accel_z;
                pitch_rate_cmd = -pitch_accel_z / speed;
            } else {
                if (switch_time_s_ < 0.0) {
                    double factor = 1.0;
                    const double elapsed = state.end_time_s - start_time_s_;
                    if (elapsed < 10.0 * time_constant_s) {
                        factor = std::max(1.0 - std::exp(-elapsed / time_constant_s), 0.01);
                    }
                    pitch_accel_z =
                        factor * (pitch_rate_rad_s * speed) * detail::sign_of(1.0, -pitch_delta);
                    cmd_accel_z = pitch_accel_z - grav_accel_z;
                    if (detail::flight_path_angle_achieved(state.vehicle.flight_path_rad,
                                                           last_flight_path_angle_rad_,
                                                           commanded_fpa)) {
                        switch_time_s_ = state.current_time_s;
                        switch_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
                        switch_lateral_acceleration_mps2_ = cmd_accel_z;
                    }
                } else {
                    const double factor = 1.0 - ((state.current_time_s - switch_time_s_) / time_constant_s);
                    if (factor <= 0.0) {
                        result.status = guidance_program_status::complete;
                        last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
                        return result;
                    }
                    cmd_accel_z = factor * switch_lateral_acceleration_mps2_;
                    pitch_accel_z = cmd_accel_z + grav_accel_z;
                }
                pitch_rate_cmd = -pitch_accel_z / speed;
            }
        } else {
            if (detail::flight_path_angle_achieved(state.vehicle.flight_path_rad,
                                                   last_flight_path_angle_rad_,
                                                   commanded_fpa)) {
                result.status = guidance_program_status::complete;
                last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
                return result;
            }

            double end_time = start_time_s_ + (std::abs(start_flight_path_angle_rad_ - commanded_fpa) / pitch_rate_rad_s);
            double time_to_go = end_time - state.current_time_s;
            if (time_to_go <= 0.0) {
                time_to_go = state.step_dt_s();
            }

            pitch_rate_cmd = pitch_delta / time_to_go;
            pitch_accel_z = -pitch_rate_cmd * speed;
            cmd_accel_z = pitch_accel_z - grav_accel_z;
        }

        result.commands.accel_cmd_ecs.z = cmd_accel_z;
        result.commands.angle_rate_cmd_rad_s.y = pitch_rate_cmd;
        detail::limit_program_accel(result.commands, phase);
        last_flight_path_angle_rad_ = state.vehicle.flight_path_rad;
        XSF_LOG_DEBUG("FPA program: cmd_gamma={:.4f} current_gamma={:.4f} az={:.3f} pitch_rate={:.4f} status={}",
                      commanded_fpa,
                      state.vehicle.flight_path_rad,
                      result.commands.accel_cmd_ecs.z,
                      result.commands.angle_rate_cmd_rad_s.y,
                      result.status == guidance_program_status::complete ? "complete" : "running");
        return result;
    }

private:
    double start_time_s_ = 0.0;
    double start_flight_path_angle_rad_ = 0.0;
    double last_flight_path_angle_rad_ = 0.0;
    double switch_time_s_ = -1.0;
    double switch_flight_path_angle_rad_ = 0.0;
    double switch_lateral_acceleration_mps2_ = 0.0;
    bool first_execute_call_ = true;
    bool pitch_down_program_ = true;
};

class orbit_insertion_program {
public:
    double ascent_gravity_bias = 0.0;
    double maximum_lateral_acceleration_mps2 = 0.1 * constants::gravity_mps2;
    double minimum_insertion_altitude_m = 100000.0;
    double coarse_adjustment_threshold_rad = 0.5 * constants::deg_to_rad;
    double fine_adjustment_threshold_rad = 0.05 * constants::deg_to_rad;

    guidance_program_result compute(const guidance_program_state& state) {
        guidance_program_result result;
        if (orbit_declared_) {
            return result;
        }

        if (state.vehicle.altitude_m <= minimum_insertion_altitude_m) {
            detail::apply_gravity_bias(state, ascent_gravity_bias, result.commands);
            return result;
        }

        const double dt_s = std::max(state.step_dt_s(), 1.0e-6);
        const vec3 vel = state.vehicle.velocity_wcs;
        const vec3 loc = state.vehicle.position_wcs;
        const double speed = vel.magnitude();
        const double radius = std::max(loc.magnitude(), 1.0);
        const double gravity_mps2 = mu_earth / (radius * radius);
        const double centripetal_accel_mps2 = (speed * speed) / radius;

        const double cos_loc_vel = std::clamp(loc.dot(vel) / (radius * std::max(speed, 1.0e-9)), -1.0, 1.0);
        const double flight_path_angle_rad = constants::half_pi - std::acos(cos_loc_vel);
        const double orbital_speed = std::sqrt(mu_earth / radius);
        const double speed_fraction = speed / orbital_speed;

        bool orbit_now_declared = false;
        double gravity_bias = 0.0;
        double gravity_bias_mod = 0.0;

        if (flight_path_angle_rad > 0.0) {
            gravity_bias = ascent_gravity_bias;
            if (flight_path_angle_rad <= coarse_adjustment_threshold_rad) {
                gravity_bias = 0.0;
            }

            if (speed_fraction < 1.0) {
                fine_adjustment_active_ = false;
                if (flight_path_angle_rad <= fine_adjustment_threshold_rad) {
                    fine_adjustment_active_ = true;
                    gravity_bias = (gravity_mps2 - centripetal_accel_mps2) / gravity_mps2;
                    gravity_bias_mod =
                        -std::min((flight_path_angle_rad / dt_s) * speed, maximum_lateral_acceleration_mps2) / gravity_mps2;
                }
            } else {
                gravity_bias = 0.0;
                if (fine_adjustment_active_) {
                    orbit_now_declared = true;
                }
            }
        } else {
            if (speed_fraction < 1.0) {
                fine_adjustment_active_ = false;
                if (flight_path_angle_rad > -fine_adjustment_threshold_rad) {
                    fine_adjustment_active_ = true;
                    gravity_bias = (gravity_mps2 - centripetal_accel_mps2) / gravity_mps2;
                    gravity_bias_mod =
                        std::min((-flight_path_angle_rad / dt_s) * speed, maximum_lateral_acceleration_mps2) / gravity_mps2;
                } else {
                    gravity_bias = std::min((-flight_path_angle_rad / dt_s) * speed, maximum_lateral_acceleration_mps2) /
                                   gravity_mps2;
                }
            } else {
                if (fine_adjustment_active_) {
                    orbit_now_declared = true;
                    gravity_bias = 0.0;
                } else {
                    gravity_bias = std::min((-flight_path_angle_rad / dt_s) * speed, maximum_lateral_acceleration_mps2) /
                                   gravity_mps2;
                }
            }
        }

        gravity_bias = std::max(gravity_bias + gravity_bias_mod, 0.0);
        detail::apply_gravity_bias(state, gravity_bias, result.commands);
        result.commands.saturated = false;
        if (orbit_now_declared) {
            orbit_declared_ = true;
            result.status = guidance_program_status::complete;
        }
        XSF_LOG_INFO("orbit insertion: alt={:.1f}m speed_frac={:.4f} fpa={:.6f}rad bias={:.4f} status={}",
                     state.vehicle.altitude_m,
                     speed_fraction,
                     flight_path_angle_rad,
                     gravity_bias,
                     result.status == guidance_program_status::complete ? "complete" : "running");
        return result;
    }

private:
    bool fine_adjustment_active_ = false;
    bool orbit_declared_ = false;
};

} // namespace xsf_math
