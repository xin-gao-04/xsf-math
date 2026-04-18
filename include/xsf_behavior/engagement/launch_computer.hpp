#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/core/vec3.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>
#include <cmath>

namespace xsf_math {

// 交战行为层：发射计算机控制器。
//
// 参考 xsf-core XsfLaunchComputer：
// - 对每条“候选目标”给出 TOF、预测拦截点、约束检查、LAR 门限结论；
// - 使用 validity flag 指出哪些字段可信，上层按需读取；
// - 约束项参考 XsfLaunchComputer::Constraints（Δ 高度 / 斜距 / 离轴角 / 速度闭合）。
// 本模块只做“是否可以打”的判定与运动学预测，不真正发射武器。

namespace launch_computer_validity {
    constexpr unsigned int launch_time        = 0x0001u;
    constexpr unsigned int launcher_bearing   = 0x0002u;
    constexpr unsigned int launcher_elevation = 0x0004u;
    constexpr unsigned int time_of_flight     = 0x0008u;
    constexpr unsigned int intercept_time     = 0x0010u;
    constexpr unsigned int intercept_point    = 0x0020u;
}  // namespace launch_computer_validity

struct launch_constraint_limits {
    // 未启用的约束保持在默认“极限”值，对应 ResultIsValidFor 的禁用语义。
    double max_delta_altitude_m = 1.0e6;
    double min_delta_altitude_m = -1.0e6;
    double max_slant_range_m    = 1.0e9;
    double min_slant_range_m    = 0.0;
    double max_boresight_rad    = constants::pi;     // 默认不限制
    double max_opening_speed_mps = 1.0e6;
    double min_opening_speed_mps = -1.0e6;
    double max_time_of_flight_s  = 1.0e6;
};

// 发射计算机所需的武器/发射平台运动学参数（来自 xsf-core 的推力+巡航段近似）。
struct weapon_kinematics {
    double avg_thrusting_accel_mps2 = 150.0;  // 推力段平均加速度
    double burnout_speed_mps        = 900.0;  // 燃尽速度
    double burn_duration_s          = 6.0;    // 推力段时长
    double coast_accel_mps2         = -2.0;   // 滑行段（含气动阻力）
    double min_terminal_speed_mps   = 200.0;
};

// 目标状态：通常来自 track_manager 的某条确认航迹。
struct launch_candidate {
    vec3   launcher_position_wcs{};    // 发射平台位置
    vec3   launcher_forward_wcs{0, 0, 0};  // 发射平台前向，用于离轴角；为零则不检查
    vec3   target_position_wcs{};
    vec3   target_velocity_wcs{};
    vec3   target_acceleration_wcs{};
};

struct launch_computer_result {
    unsigned int validity_flags = 0u;
    double       time_of_flight_s = 0.0;
    double       intercept_time_s = 0.0;
    vec3         intercept_point_wcs{};
    double       launcher_bearing_rad   = 0.0;
    double       launcher_elevation_rad = 0.0;

    bool all_constraints_met = false;
    // 违反哪几项约束；与 launch_constraint_limits 对齐。
    bool violated_delta_altitude = false;
    bool violated_slant_range    = false;
    bool violated_boresight      = false;
    bool violated_opening_speed  = false;
    bool violated_tof            = false;

    bool is_valid(unsigned int flag) const { return (validity_flags & flag) != 0u; }
};

struct launch_computer {
    weapon_kinematics      weapon{};
    launch_constraint_limits limits{};
    int intercept_refine_iterations = 4;

    // 估算武器平均速度：简单两段平均（推力末速和最小末速的算术均值）。
    double nominal_flight_speed_mps() const {
        double avg = 0.5 * (weapon.burnout_speed_mps + weapon.min_terminal_speed_mps);
        return std::max(avg, 1.0);
    }

    double distance_travelled_by_weapon(double time_s) const {
        if (time_s <= 0.0) return 0.0;

        double burn_time = std::max(weapon.burn_duration_s, 1.0e-6);
        double accel = std::max(weapon.avg_thrusting_accel_mps2,
                                weapon.burnout_speed_mps / burn_time);
        if (time_s <= burn_time) return 0.5 * accel * time_s * time_s;

        double burn_distance = 0.5 * accel * burn_time * burn_time;
        double coast_time = time_s - burn_time;
        if (weapon.coast_accel_mps2 >= 0.0) {
            return burn_distance + weapon.burnout_speed_mps * coast_time +
                   0.5 * weapon.coast_accel_mps2 * coast_time * coast_time;
        }

        double time_to_terminal =
            (weapon.min_terminal_speed_mps - weapon.burnout_speed_mps) / weapon.coast_accel_mps2;
        time_to_terminal = std::max(time_to_terminal, 0.0);
        if (coast_time <= time_to_terminal) {
            return burn_distance + weapon.burnout_speed_mps * coast_time +
                   0.5 * weapon.coast_accel_mps2 * coast_time * coast_time;
        }

        double decel_distance = weapon.burnout_speed_mps * time_to_terminal +
                                0.5 * weapon.coast_accel_mps2 * time_to_terminal * time_to_terminal;
        return burn_distance + decel_distance +
               weapon.min_terminal_speed_mps * (coast_time - time_to_terminal);
    }

    double solve_time_to_range(double range_m) const {
        if (range_m <= 0.0) return 0.0;

        double hi = std::min(limits.max_time_of_flight_s, 10.0);
        while (distance_travelled_by_weapon(hi) < range_m && hi < limits.max_time_of_flight_s) {
            hi = std::min(hi * 2.0, limits.max_time_of_flight_s);
            if (hi >= limits.max_time_of_flight_s) break;
        }
        if (distance_travelled_by_weapon(hi) < range_m) return limits.max_time_of_flight_s;

        double lo = 0.0;
        for (int i = 0; i < 40; ++i) {
            double mid = 0.5 * (lo + hi);
            if (distance_travelled_by_weapon(mid) >= range_m) hi = mid;
            else lo = mid;
        }
        return hi;
    }

    // 基于“等速追击 + 目标线性外推”的二次方程近似拦截时间。
    // 与 XsfLaunchComputer::EstimatedTimeToIntercept 的工程估算思路一致。
    launch_computer_result evaluate(const launch_candidate& cand, double sim_time_s) const {
        launch_computer_result r;

        vec3 rel_pos = cand.target_position_wcs - cand.launcher_position_wcs;
        double R = rel_pos.magnitude();
        double v_wpn = nominal_flight_speed_mps();
        double v_tgt = cand.target_velocity_wcs.magnitude();

        // 二次方程： |R + V_t * t|^2 = (v_wpn * t)^2
        double a = v_tgt * v_tgt - v_wpn * v_wpn;
        double b = 2.0 * rel_pos.dot(cand.target_velocity_wcs);
        double c = R * R;
        double tof = 0.0;

        if (std::abs(a) < 1e-6) {
            tof = (std::abs(b) > 1e-6) ? -c / b : (R / v_wpn);
        } else {
            double disc = b * b - 4.0 * a * c;
            if (disc < 0.0) {
                XSF_LOG_DEBUG("launch computer: no real intercept (disc<0) R={:.0f}m", R);
                return r;
            }
            double sq = std::sqrt(disc);
            double t1 = (-b + sq) / (2.0 * a);
            double t2 = (-b - sq) / (2.0 * a);
            if (t1 > 0.0 && t2 > 0.0) tof = std::min(t1, t2);
            else if (t1 > 0.0)        tof = t1;
            else if (t2 > 0.0)        tof = t2;
            else {
                XSF_LOG_DEBUG("launch computer: no positive tof for R={:.0f}m", R);
                return r;
            }
        }

        vec3 impact{};
        for (int iter = 0; iter < std::max(intercept_refine_iterations, 1); ++iter) {
            impact = cand.target_position_wcs +
                     cand.target_velocity_wcs * tof +
                     0.5 * cand.target_acceleration_wcs * tof * tof;
            double refined_range = (impact - cand.launcher_position_wcs).magnitude();
            double refined_tof = solve_time_to_range(refined_range);
            if (std::abs(refined_tof - tof) < 1.0e-2) {
                tof = refined_tof;
                break;
            }
            tof = refined_tof;
        }
        impact = cand.target_position_wcs +
                 cand.target_velocity_wcs * tof +
                 0.5 * cand.target_acceleration_wcs * tof * tof;

        r.time_of_flight_s = tof;
        r.intercept_time_s = sim_time_s + tof;
        r.intercept_point_wcs = impact;
        r.validity_flags |= launch_computer_validity::launch_time;
        r.validity_flags |= launch_computer_validity::time_of_flight;
        r.validity_flags |= launch_computer_validity::intercept_time;
        r.validity_flags |= launch_computer_validity::intercept_point;

        vec3 aim = impact - cand.launcher_position_wcs;
        double ground = std::sqrt(aim.x * aim.x + aim.y * aim.y);
        r.launcher_bearing_rad   = std::atan2(aim.y, aim.x);
        r.launcher_elevation_rad = std::atan2(-aim.z, ground);
        r.validity_flags |= launch_computer_validity::launcher_bearing;
        r.validity_flags |= launch_computer_validity::launcher_elevation;

        // 约束检查
        double delta_alt = -aim.z;  // WCS Z 向下
        double opening   = -cand.target_velocity_wcs.dot(rel_pos) / std::max(R, 1e-6);

        r.violated_slant_range    = (R > limits.max_slant_range_m) || (R < limits.min_slant_range_m);
        r.violated_delta_altitude = (delta_alt > limits.max_delta_altitude_m) ||
                                     (delta_alt < limits.min_delta_altitude_m);
        r.violated_opening_speed  = (opening > limits.max_opening_speed_mps) ||
                                     (opening < limits.min_opening_speed_mps);
        r.violated_tof            = (tof > limits.max_time_of_flight_s);

        double fwd_len = cand.launcher_forward_wcs.magnitude();
        if (fwd_len > 1e-6) {
            vec3 aim_unit = aim.normalized();
            vec3 fwd_unit = cand.launcher_forward_wcs / fwd_len;
            double cos_ang = std::clamp(aim_unit.dot(fwd_unit), -1.0, 1.0);
            double boresight = std::acos(cos_ang);
            r.violated_boresight = (boresight > limits.max_boresight_rad);
        }

        r.all_constraints_met = !(r.violated_slant_range || r.violated_delta_altitude ||
                                   r.violated_opening_speed || r.violated_tof ||
                                   r.violated_boresight);
        XSF_LOG_DEBUG("launch computer: R={:.0f}m tof={:.2f}s intercept=[{:.0f},{:.0f},{:.0f}] ok={}",
                      R, tof, impact.x, impact.y, impact.z, r.all_constraints_met);
        return r;
    }
};

} // namespace xsf_math
