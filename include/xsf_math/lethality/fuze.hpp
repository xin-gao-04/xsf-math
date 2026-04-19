#pragma once

#include "../core/vec3.hpp"
#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 近炸引信与杀伤效能模型（Proximity fuze and lethality model）

// 引信起爆结果枚举（Fuze detonation result enumeration）
enum class fuze_result {
    no_trigger,       // 未触发（仍在搜索）（No trigger, still searching）
    proximity_burst,  // 近炸起爆（Proximity burst）
    contact,          // 直接命中（Direct contact）
    miss,             // 目标已通过，引信超时（Target passed, fuze timeout）
    dud               // 引信故障（Dud）
};

// 最近接近点（Closest Point of Approach, CPA）计算结果 (CPA calculation result)
struct cpa_result {
    double miss_distance_m = 1e10;  // 最近接近距离（Miss distance），单位 m
    double time_to_cpa_s   = 0.0;   // 距离 CPA 的时间（Time to CPA），负值表示已越过，单位 s
    vec3   cpa_point_weapon;         // 武器在 CPA 时的位置（Weapon position at CPA）
    vec3   cpa_point_target;         // 目标在 CPA 时的位置（Target position at CPA）
};

// 假设武器和目标速度恒定，计算 CPA（Compute CPA assuming constant velocities for weapon and target）
inline cpa_result compute_cpa(const vec3& weapon_pos, const vec3& weapon_vel,
                               const vec3& target_pos, const vec3& target_vel) {
    vec3 dr = target_pos - weapon_pos;  // 相对位置矢量（Relative position vector）
    vec3 dv = target_vel - weapon_vel;  // 相对速度矢量（Relative velocity vector）
    double dv2 = dv.magnitude_sq();

    cpa_result r;
    if (dv2 < 1e-20) {
        // 无相对运动（No relative motion）
        r.miss_distance_m = dr.magnitude();
        r.time_to_cpa_s = 0.0;
    } else {
        r.time_to_cpa_s = -dr.dot(dv) / dv2;
        vec3 closest_dr = dr + dv * r.time_to_cpa_s;
        r.miss_distance_m = closest_dr.magnitude();
    }

    r.cpa_point_weapon = weapon_pos + weapon_vel * r.time_to_cpa_s;
    r.cpa_point_target = target_pos + target_vel * r.time_to_cpa_s;

    return r;
}

// 近炸引信模型（Proximity fuze model）
struct proximity_fuze {
    double trigger_radius_m = 10.0;    // 近炸触发距离（Trigger radius），单位 m
    double arm_delay_s      = 1.0;     // 发射后解除保险的延迟（Arming delay after launch），单位 s
    double arm_range_m      = 500.0;   // 解除保险前的最小距离（Minimum distance before arming），单位 m
    double dud_probability  = 0.0;     // 哑弹概率（Dud probability），范围 [0,1]

    // 检查引信触发条件（Check fuze trigger conditions）
    // flight_time_s: 飞行时间（s）; weapon_pos/target_pos: 位置（m）; weapon_vel/target_vel: 速度（m/s）; dt_s: 时间步长（s）
    fuze_result check(double flight_time_s,
                      const vec3& weapon_pos, const vec3& weapon_vel,
                      const vec3& target_pos, const vec3& target_vel,
                      double dt_s) const {

        // 检查解除保险条件（Check arming condition）
        if (flight_time_s < arm_delay_s) return fuze_result::no_trigger;

        double range = (target_pos - weapon_pos).magnitude();
        // 注意：arm_range_m 表示距发射点的最小距离，不是距目标的距离
        // Note: arm_range_m is minimum distance from launch point, not from target

        // CPA 检查（CPA check）
        auto cpa = compute_cpa(weapon_pos, weapon_vel, target_pos, target_vel);

        // 接触判定（极近距离）（Contact判定: extremely close range）
        if (range < 1.0) return fuze_result::contact;

        // 近炸触发（Proximity burst trigger）
        if (range <= trigger_radius_m && cpa.time_to_cpa_s <= dt_s) {
            return fuze_result::proximity_burst;
        }

        // 检查目标是否已被越过（CPA 在过去）（Check if target has been passed, CPA in the past）
        if (cpa.time_to_cpa_s < -dt_s && cpa.miss_distance_m > trigger_radius_m) {
            return fuze_result::miss;
        }

        return fuze_result::no_trigger;
    }
};

// 两阶段 PCA（Proximity Check Algorithm，近炸检查算法）
// 第 1 阶段：粗门限（大半径，快速检查）
// 第 2 阶段：细门限（精确距离计算）
struct pca_two_stage {
    double coarse_gate_m = 100.0;  // 第 1 阶段粗门限半径（Coarse gate radius），单位 m
    double fine_gate_m   = 10.0;   // 第 2 阶段细门限半径（Fine gate radius），单位 m

    // 近炸检查结果（Proximity check result）
    struct result {
        bool   in_coarse_gate = false;  // 是否在粗门限内（In coarse gate）
        bool   in_fine_gate   = false;  // 是否在细门限内（In fine gate）
        double distance_m     = 1e10;   // 武器与目标距离（Distance），单位 m
    };

    // 执行两阶段近炸检查（Perform two-stage proximity check）
    result check(const vec3& weapon_pos, const vec3& target_pos) const {
        result r;
        vec3 diff = target_pos - weapon_pos;
        r.distance_m = diff.magnitude();
        r.in_coarse_gate = (r.distance_m <= coarse_gate_m);
        r.in_fine_gate   = (r.distance_m <= fine_gate_m);
        return r;
    }
};

} // namespace xsf_math
