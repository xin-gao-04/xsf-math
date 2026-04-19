#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/orbital/lambert.hpp>
#include <xsf_math/orbital/maneuvers.hpp>
#include <vector>

namespace xsf_math {

// 轨道行为层：机动序列规划器（Orbital behavior layer: maneuver sequence planner）。
//
// 参考 xsf-core XsfOrbitalManeuversHohmannTransfer 的组织：
// - 高升轨：在近地点点火建立转移椭圆，在远地点圆化；
// - 低降轨：在远地点点火建立转移椭圆，在近地点圆化；
// - 平面机动与 Hohmann 可以串联成复合机动序列。
// 本规划器不传播轨道，只给出每一步的点火条件和 Δv 估算，供外部传播器驱动。
// Reference xsf-core XsfOrbitalManeuversHohmannTransfer organization:
// - Raise orbit: burn at periapsis to establish transfer ellipse, circularize at apoapsis;
// - Lower orbit: burn at apoapsis to establish transfer ellipse, circularize at periapsis;
// - Plane changes and Hohmann can be chained into composite maneuver sequences.
// This planner does not propagate orbits; it only gives ignition conditions and Δv estimates for each step.

// 轨道点火条件（Orbital burn condition）
enum class orbital_burn_condition {
    immediate,    // 立即（Immediate）
    at_periapsis, // 在近地点（At periapsis）
    at_apoapsis   // 在远地点（At apoapsis）
};

// 轨道点火类型（Orbital burn kind）
enum class orbital_burn_kind {
    inplane_sma_change,       // 共面半长轴改变（In-plane semi-major axis change）
    plane_change,             // 平面机动（Plane change）
    raan_inclination_change,  // 升交点赤经/倾角改变（RAAN/inclination change）
    circularize,              // 圆化（Circularize）
    lambert_transfer          // Lambert 转移（Lambert transfer）
};

// 轨道点火步骤（Orbital burn step）
struct orbital_burn_step {
    orbital_burn_kind       kind = orbital_burn_kind::inplane_sma_change;  // 点火类型（Burn kind）
    orbital_burn_condition  condition = orbital_burn_condition::immediate;  // 点火条件（Burn condition）
    double                  delta_v_mps = 0.0;  // 速度增量 m/s（Delta-v in m/s）
    double                  transfer_sma_m = 0.0;       // 仅 SMA 改变使用（Transfer SMA, for SMA changes only）
    double                  target_inclination_rad = 0.0; // 仅平面机动（Target inclination, for plane changes only）
    double                  target_raan_rad = 0.0;        // 仅平面机动（Target RAAN, for plane changes only）
    vec3                    target_position_eci{};  // 目标位置 ECI（Target position in ECI）
};

// 轨道规划结果（Orbital plan result）
struct orbital_plan_result {
    std::vector<orbital_burn_step> steps;  // 点火步骤列表（Burn steps）
    double total_delta_v_mps = 0.0;  // 总速度增量 m/s（Total delta-v in m/s）
    double total_duration_s  = 0.0;  // 总持续时间秒（Total duration in seconds）
};

// 轨道初始状态（Orbital initial state）
struct orbital_initial_state {
    double semi_major_axis_m = 0.0;  // 半长轴米（Semi-major axis in meters）
    double eccentricity      = 0.0;  // 偏心率（Eccentricity）
    double speed_mps         = 0.0;  // 速度 m/s（Speed in m/s）
    double inclination_rad   = 0.0;  // 倾角弧度（Inclination in radians）
    double raan_rad          = 0.0;  // 升交点赤经弧度（RAAN in radians）
    double mu                = mu_earth;  // 引力参数（Gravitational parameter）
};

// 轨道机动规划器（Orbital maneuver planner）
struct orbital_maneuver_planner {
    // Hohmann 转移：在当前近/远点点火，到达远/近点圆化（Hohmann transfer: burn at periapsis/apoapsis, circularize at opposite apsis）。
    // 参考 xsf-core HohmannTransfer::Initialize 对 transferSMA 的分支选择（Reference xsf-core HohmannTransfer::Initialize transferSMA branching）。
    orbital_plan_result plan_hohmann(const orbital_initial_state& s0,
                                      double final_sma_m) const {
        orbital_plan_result out;
        if (s0.semi_major_axis_m <= 0.0 || final_sma_m <= 0.0) return out;

        orbital_burn_step first;
        orbital_burn_step second;
        double transfer_sma = 0.0;

        if (final_sma_m > s0.semi_major_axis_m) {
            // 升轨：近地点点火建立转移椭圆，远地点二次点火圆化（Raise orbit: periapsis burn for transfer ellipse, apoapsis circularization）
            transfer_sma = 0.5 * (final_sma_m + s0.semi_major_axis_m * (1.0 - s0.eccentricity));
            first.condition  = orbital_burn_condition::at_periapsis;
            second.condition = orbital_burn_condition::at_apoapsis;
        } else {
            transfer_sma = 0.5 * (final_sma_m + s0.semi_major_axis_m * (1.0 + s0.eccentricity));
            first.condition  = orbital_burn_condition::at_apoapsis;
            second.condition = orbital_burn_condition::at_periapsis;
        }

        // 用工程简化的 hohmann_transfer 给出两次 Δv 估算（Use simplified hohmann_transfer for two-burn Δv estimates）
        auto ht = hohmann_transfer(s0.semi_major_axis_m, final_sma_m, s0.mu);

        first.kind = orbital_burn_kind::inplane_sma_change;
        first.transfer_sma_m = transfer_sma;
        first.delta_v_mps = std::abs(ht.delta_v1_mps);

        second.kind = orbital_burn_kind::circularize;
        second.transfer_sma_m = final_sma_m;
        second.delta_v_mps = std::abs(ht.delta_v2_mps);

        out.steps.push_back(first);
        out.steps.push_back(second);
        out.total_delta_v_mps = ht.total_delta_v_mps;
        out.total_duration_s  = ht.transfer_time_s;

        XSF_LOG_DEBUG("orbital planner: Hohmann a0={:.0f} a1={:.0f} ΔvTot={:.1f}m/s T={:.0f}s",
                      s0.semi_major_axis_m, final_sma_m,
                      out.total_delta_v_mps, out.total_duration_s);
        return out;
    }

    // 升轨 + 改变倾角的复合机动：Hohmann 前加一次 RAAN/inclination 合并机动（Combined raise + plane change: add RAAN/inclination burn before Hohmann）
    orbital_plan_result plan_hohmann_with_plane_change(const orbital_initial_state& s0,
                                                       double final_sma_m,
                                                       double final_inclination_rad,
                                                       double final_raan_rad) const {
        orbital_plan_result out = plan_hohmann(s0, final_sma_m);
        if (out.steps.empty()) return out;

        orbital_burn_step plane;
        plane.kind = orbital_burn_kind::raan_inclination_change;
        plane.condition = orbital_burn_condition::immediate;
        plane.delta_v_mps = raan_inclination_change_delta_v(
            s0.speed_mps, s0.inclination_rad, s0.raan_rad,
            final_inclination_rad, final_raan_rad);
        plane.target_inclination_rad = final_inclination_rad;
        plane.target_raan_rad        = final_raan_rad;

        out.steps.insert(out.steps.begin(), plane);
        out.total_delta_v_mps += plane.delta_v_mps;
        return out;
    }

    // 单独的圆化机动（Standalone circularization maneuver）
    orbital_plan_result plan_circularize(const orbital_initial_state& s0, double radius_m) const {
        orbital_plan_result out;
        orbital_burn_step b;
        b.kind = orbital_burn_kind::circularize;
        b.condition = orbital_burn_condition::immediate;
        b.delta_v_mps = std::abs(circularize_delta_v(radius_m, s0.speed_mps, s0.mu));
        b.transfer_sma_m = radius_m;
        out.steps.push_back(b);
        out.total_delta_v_mps = b.delta_v_mps;
        return out;
    }

    orbital_plan_result plan_lambert_intercept(const vec3& current_position_eci,
                                               const vec3& current_velocity_eci,
                                               const vec3& target_position_eci,
                                               double time_of_flight_s,
                                               double gravitational_parameter = mu_earth) const {
        orbital_plan_result out;
        auto lambert = solve_lambert(current_position_eci,
                                     target_position_eci,
                                     time_of_flight_s,
                                     false,
                                     gravitational_parameter);
        if (!lambert.valid) return out;

        orbital_burn_step b;
        b.kind = orbital_burn_kind::lambert_transfer;
        b.condition = orbital_burn_condition::immediate;
        b.delta_v_mps = (lambert.departure_velocity_eci - current_velocity_eci).magnitude();
        b.target_position_eci = target_position_eci;
        out.steps.push_back(b);
        out.total_delta_v_mps = b.delta_v_mps;
        out.total_duration_s = time_of_flight_s;
        return out;
    }
};

} // namespace xsf_math
