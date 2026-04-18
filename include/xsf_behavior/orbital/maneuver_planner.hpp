#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/orbital/lambert.hpp>
#include <xsf_math/orbital/maneuvers.hpp>
#include <vector>

namespace xsf_math {

// 轨道行为层：机动序列规划器。
//
// 参考 xsf-core XsfOrbitalManeuversHohmannTransfer 的组织：
// - 高升轨：在近地点点火建立转移椭圆，在远地点圆化；
// - 低降轨：在远地点点火建立转移椭圆，在近地点圆化；
// - 平面机动与 Hohmann 可以串联成复合机动序列。
// 本规划器不传播轨道，只给出每一步的点火条件和 Δv 估算，供外部传播器驱动。

enum class orbital_burn_condition {
    immediate,
    at_periapsis,
    at_apoapsis
};

enum class orbital_burn_kind {
    inplane_sma_change,
    plane_change,
    raan_inclination_change,
    circularize,
    lambert_transfer
};

struct orbital_burn_step {
    orbital_burn_kind       kind = orbital_burn_kind::inplane_sma_change;
    orbital_burn_condition  condition = orbital_burn_condition::immediate;
    double                  delta_v_mps = 0.0;
    double                  transfer_sma_m = 0.0;       // 仅 SMA 改变使用
    double                  target_inclination_rad = 0.0; // 仅平面机动
    double                  target_raan_rad = 0.0;        // 仅平面机动
    vec3                    target_position_eci{};
};

struct orbital_plan_result {
    std::vector<orbital_burn_step> steps;
    double total_delta_v_mps = 0.0;
    double total_duration_s  = 0.0;
};

struct orbital_initial_state {
    double semi_major_axis_m = 0.0;
    double eccentricity      = 0.0;
    double speed_mps         = 0.0;
    double inclination_rad   = 0.0;
    double raan_rad          = 0.0;
    double mu                = mu_earth;
};

struct orbital_maneuver_planner {
    // Hohmann 转移：在当前近/远点点火，到达远/近点圆化。
    // 参考 xsf-core HohmannTransfer::Initialize 对 transferSMA 的分支选择。
    orbital_plan_result plan_hohmann(const orbital_initial_state& s0,
                                      double final_sma_m) const {
        orbital_plan_result out;
        if (s0.semi_major_axis_m <= 0.0 || final_sma_m <= 0.0) return out;

        orbital_burn_step first;
        orbital_burn_step second;
        double transfer_sma = 0.0;

        if (final_sma_m > s0.semi_major_axis_m) {
            // 升轨：近地点点火建立转移椭圆，远地点二次点火圆化。
            transfer_sma = 0.5 * (final_sma_m + s0.semi_major_axis_m * (1.0 - s0.eccentricity));
            first.condition  = orbital_burn_condition::at_periapsis;
            second.condition = orbital_burn_condition::at_apoapsis;
        } else {
            transfer_sma = 0.5 * (final_sma_m + s0.semi_major_axis_m * (1.0 + s0.eccentricity));
            first.condition  = orbital_burn_condition::at_apoapsis;
            second.condition = orbital_burn_condition::at_periapsis;
        }

        // 用工程简化的 hohmann_transfer 给出两次 Δv 估算。
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

    // 升轨 + 改变倾角的复合机动：Hohmann 前加一次 RAAN/inclination 合并机动。
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

    // 单独的圆化机动。
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
