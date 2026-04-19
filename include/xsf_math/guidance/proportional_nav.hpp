#pragma once

#include "../core/vec3.hpp"
#include "../core/constants.hpp"
#include <xsf_common/log.hpp>
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 武器与目标之间的相对几何关系（Engagement Geometry between Weapon and Target）
struct engagement_geometry {
    vec3   weapon_pos;       // 武器位置（WCS）（Weapon Position in WCS）
    vec3   weapon_vel;       // 武器速度（WCS）（Weapon Velocity in WCS）
    vec3   target_pos;       // 目标位置（WCS）（Target Position in WCS）
    vec3   target_vel;       // 目标速度（WCS）（Target Velocity in WCS）
    vec3   target_accel;     // 目标加速度估计（WCS），用于 APN（Target Acceleration Estimate in WCS, for APN）

    // 派生量（Derived Quantities）
    vec3 relative_pos() const { return target_pos - weapon_pos; }  // 相对位置（Relative Position）
    vec3 relative_vel() const { return target_vel - weapon_vel; }  // 相对速度（Relative Velocity）

    double slant_range() const { return relative_pos().magnitude(); }  // 斜距（Slant Range）

    // 闭合速度（接近时为正）（Closing Velocity, Positive when Approaching）
    double closing_velocity() const {
        vec3 rp = relative_pos();
        double R = rp.magnitude();
        if (R < 1e-10) return 0.0;
        return -relative_vel().dot(rp) / R;
    }

    // 视线单位向量（Line-of-Sight Unit Vector）
    vec3 los_unit() const { return relative_pos().normalized(); }

    // 视线角速度向量：omega = (R x V_rel) / |R|^2（Line-of-Sight Rate Vector: omega = (R x V_rel) / |R|^2）
    vec3 los_rate() const {
        vec3 R = relative_pos();
        double R2 = R.magnitude_sq();
        if (R2 < 1e-20) return {0, 0, 0};
        return R.cross(relative_vel()) / R2;
    }

    // 命中时间估计（假设闭合速度恒定）（Estimated Time to Intercept, Assuming Constant Closing Velocity）
    double time_to_intercept() const {
        double vc = closing_velocity();
        if (vc <= 0.0) return 1e10;  // 发散
        return slant_range() / vc;
    }

    // 地面距离（水平距离）（Ground Range, Horizontal Distance）
    double ground_range() const {
        vec3 r = relative_pos();
        return std::sqrt(r.x*r.x + r.y*r.y);
    }

    // 视线方位角和俯仰角（Line-of-Sight Azimuth and Elevation Angles）
    double los_azimuth() const {
        vec3 r = relative_pos();
        return std::atan2(r.y, r.x);
    }

    double los_elevation() const {
        vec3 r = relative_pos();
        double gr = std::sqrt(r.x*r.x + r.y*r.y);
        return std::atan2(-r.z, gr);  // Z 轴向下（Z-axis Points Downwards）
    }
};

// 纯比例导引（PN）（Pure Proportional Navigation, PN）
// a_cmd = N * V_c * dtheta/dt  （标量形式）
// a_cmd = N * (omega x V_wpn)   （三维向量形式）
struct proportional_nav {
    double nav_ratio = 3.0;  // PN 导引比（PN Navigation Ratio）

    // 计算加速度指令（Compute Acceleration Command）
    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 omega = geom.los_rate();
        vec3 accel = nav_ratio * omega.cross(geom.weapon_vel);
        XSF_LOG_TRACE("PN: N={:.2f} range={:.1f}m Vc={:.1f}m/s |omega|={:.6f}rad/s |a|={:.3f}m/s^2",
                      nav_ratio,
                      geom.slant_range(),
                      geom.closing_velocity(),
                      omega.magnitude(),
                      accel.magnitude());
        return accel;
    }
};

// 增广比例导引（APN）（Augmented Proportional Navigation, APN）
// a_cmd = N * (omega x V_wpn) + (N/2) * a_target
struct augmented_proportional_nav {
    double nav_ratio = 3.0;  // APN 导引比（APN Navigation Ratio）

    // 计算加速度指令（Compute Acceleration Command）
    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 omega = geom.los_rate();
        vec3 pn_accel = nav_ratio * omega.cross(geom.weapon_vel);
        // 目标加速度以负半系数进入 APN 修正项。（Target Acceleration Enters APN Correction Term with a Negative Half Coefficient.）
        vec3 aug_term = (-nav_ratio / 2.0) * geom.target_accel;
        vec3 accel = pn_accel + aug_term;
        XSF_LOG_TRACE("APN: N={:.2f} range={:.1f}m Vc={:.1f}m/s |pn|={:.3f}m/s^2 |aug|={:.3f}m/s^2 |a|={:.3f}m/s^2",
                      nav_ratio,
                      geom.slant_range(),
                      geom.closing_velocity(),
                      pn_accel.magnitude(),
                      aug_term.magnitude(),
                      accel.magnitude());
        return accel;
    }
};

// 追踪导引（Pursuit Guidance）
// 将速度向量朝向目标当前方位（Aligns Velocity Vector toward Target's Current Bearing）
// a_cmd = K * sin(target_offset_angle) * g
struct pursuit_guidance {
    double gain = 10.0;  // 追踪导引增益（Pursuit Guidance Gain）

    // 计算加速度指令（Compute Acceleration Command）
    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 aim_dir = geom.relative_pos().normalized();
        vec3 vel_dir = geom.weapon_vel.normalized();
        double speed = geom.weapon_vel.magnitude();

        if (speed < 1e-10) return {0, 0, 0};

        // 双重叉乘法（避免显式三角函数）：（Double Cross Product Method, Avoiding Explicit Trigonometric Functions）:
        // n = V x AimDir  -> 机动平面的法向量（n = V x AimDir -> Normal Vector of the Maneuver Plane）
        // lateral = n x V -> 平面内的侧向方向（lateral = n x V -> Lateral Direction within the Plane）
        // |lateral| / |V|^2 = sin(offset_angle)（|lateral| / |V|^2 = sin(offset_angle)）
        vec3 n = vel_dir.cross(aim_dir);
        vec3 lateral = n.cross(vel_dir);

        double lateral_mag = lateral.magnitude();
        // 当两个输入都是单位向量时，lateral_mag = sin(offset_angle)（When Both Inputs are Unit Vectors, lateral_mag = sin(offset_angle)）

        if (lateral_mag < 1e-10) return {0, 0, 0};

        vec3 accel = lateral.normalized() * (gain * constants::gravity_mps2 * lateral_mag);
        XSF_LOG_TRACE("pursuit: gain={:.2f} range={:.1f}m offset={:.6f} |a|={:.3f}m/s^2",
                      gain,
                      geom.slant_range(),
                      lateral_mag,
                      accel.magnitude());
        return accel;
    }
};

// 制导加速度限幅器（Guidance Acceleration Limiter）
// 将指令约束到可用气动能力内（Constrains Command within Available Aerodynamic Capability）
struct accel_limiter {
    double max_g = 30.0;  // 最大可用过载（Maximum Available G-load）

    // 限幅加速度指令（Limit Acceleration Command）
    vec3 limit(const vec3& accel_cmd) const {
        double max_accel = max_g * constants::gravity_mps2;
        double mag = accel_cmd.magnitude();
        if (mag <= max_accel) return accel_cmd;
        vec3 limited = accel_cmd * (max_accel / mag);
        XSF_LOG_DEBUG("accel limited: |cmd|={:.3f}m/s^2 limit={:.3f}m/s^2 max_g={:.2f}",
                      mag,
                      max_accel,
                      max_g);
        return limited;
    }

    // 根据动压和导弹参数计算最大可用过载（Calculate Maximum Available G-load from Dynamic Pressure and Missile Parameters）
    static double max_available_g(double dynamic_pressure_pa,
                                   double ref_area_m2,
                                   double cl_max,
                                   double mass_kg) {
        if (mass_kg <= 0.0) return 0.0;
        double max_force = dynamic_pressure_pa * ref_area_m2 * cl_max;
        return max_force / (mass_kg * constants::gravity_mps2);
    }
};

// 多阶段制导状态机（Multi-Phase Guidance State Machine）
enum class guidance_phase_trigger {
    flight_time,        // 飞行时间（Flight Time）
    altitude,           // 高度（Altitude）
    speed,              // 速度（Speed）
    mach,               // 马赫数（Mach Number）
    dynamic_pressure,   // 动压（Dynamic Pressure）
    target_range,       // 目标距离（Target Range）
    ground_range,       // 地面距离（Ground Range）
    time_to_intercept,  // 拦截剩余时间（Time to Intercept）
    los_elevation,      // 视线俯仰角（Line-of-Sight Elevation）
    los_azimuth         // 视线方位角（Line-of-Sight Azimuth）
};

struct phase_transition {
    guidance_phase_trigger trigger;  // 触发类型（Trigger Type）
    double threshold;                // 阈值（Threshold）
    bool   trigger_above = true;     // true：当数值 > 阈值时触发（true: Triggered when Value Exceeds Threshold）
};

// 判断是否应发生阶段切换（Determine whether Phase Transition Should Occur）
inline bool check_phase_transition(const phase_transition& trans,
                                    const engagement_geometry& geom,
                                    double flight_time,
                                    double altitude,
                                    double speed,
                                    double mach,
                                    double dynamic_pressure) {
    double value = 0.0;
    switch (trans.trigger) {
    case guidance_phase_trigger::flight_time:
        value = flight_time; break;
    case guidance_phase_trigger::altitude:
        value = altitude; break;
    case guidance_phase_trigger::speed:
        value = speed; break;
    case guidance_phase_trigger::mach:
        value = mach; break;
    case guidance_phase_trigger::dynamic_pressure:
        value = dynamic_pressure; break;
    case guidance_phase_trigger::target_range:
        value = geom.slant_range(); break;
    case guidance_phase_trigger::ground_range:
        value = geom.ground_range(); break;
    case guidance_phase_trigger::time_to_intercept:
        value = geom.time_to_intercept(); break;
    case guidance_phase_trigger::los_elevation:
        value = geom.los_elevation(); break;
    case guidance_phase_trigger::los_azimuth:
        value = geom.los_azimuth(); break;
    }

    if (trans.trigger_above) return value >= trans.threshold;
    else                     return value <= trans.threshold;
}

} // namespace xsf_math
