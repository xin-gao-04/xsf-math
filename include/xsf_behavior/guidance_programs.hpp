#pragma once

#include "flight_state.hpp"
#include <xsf_math/core/coordinate_transform.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>
#include <algorithm>
#include <cmath>
#include <optional>

namespace xsf_math {

// 这一层按 xsf-core 的“guidance program”语义组织，而不是外部适配动作。
// 它保留程序级流程：LOS 导引、偏置、平滑、飞行路径角/高度覆盖、完成判据。

enum class guidance_program_status {
    continue_running,
    complete
};

enum class guidance_pn_method {
    pure,
    augmented
};

struct guidance_program_commands {
    vec3 accel_cmd_ecs{};           // 对齐 xsf-core 的 ECS 加速度命令
    vec3 angle_rate_cmd_rad_s{};    // 对齐 xsf-core 的姿态角速率命令
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

namespace detail {

inline double sign_of(double magnitude, double reference) {
    if (reference > 0.0) return magnitude;
    if (reference < 0.0) return -magnitude;
    return 0.0;
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

} // namespace xsf_math
