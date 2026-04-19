#pragma once

#include "../core/constants.hpp"
#include "../core/quaternion.hpp"
#include "../core/vec3.hpp"
#include <algorithm>

namespace xsf_math {

// 刚体输入（Rigid Body Inputs）
struct rigid_body_inputs {
    vec3 force_body_n{};  // 机体坐标系力，单位牛顿（Force in Body Coordinates, Newtons）
    vec3 moment_body_nm{};  // 机体坐标系力矩，单位牛米（Moment in Body Coordinates, Newton-meters）
    vec3 external_accel_wcs{};  // 世界坐标系外部加速度（External Acceleration in WCS）
    double mass_flow_kgps = 0.0;  // 质量流量，单位 kg/s（Mass Flow Rate in kg/s）
};

// 刚体状态（Rigid Body State）
struct rigid_body_state {
    vec3 position_wcs{};  // 世界坐标系位置（Position in WCS）
    vec3 velocity_wcs{};  // 世界坐标系速度（Velocity in WCS）
    quaternion attitude_body_to_wcs{};  // 机体到世界坐标系的姿态四元数（Attitude Quaternion from Body to WCS）
    vec3 body_rates_radps{};  // 机体角速度，单位 rad/s（Body Angular Rates in rad/s）
    double mass_kg = 1.0;  // 质量，单位千克（Mass in Kilograms）
    vec3 inertia_diag_kgm2{1.0, 1.0, 1.0};  // 惯性张量对角线分量，单位 kg·m²（Inertia Tensor Diagonal Components in kg·m²）

    // 计算机体坐标系速度（Compute Body-Frame Velocity）
    vec3 body_velocity_mps() const {
        return attitude_body_to_wcs.inverse().rotate(velocity_wcs);
    }
};

// 刚体状态导数（Rigid Body State Derivative）
struct rigid_body_derivative {
    vec3 position_dot_wcs{};  // 位置变化率（Position Derivative）
    vec3 velocity_dot_wcs{};  // 速度变化率（Velocity Derivative）
    quaternion attitude_dot{};  // 姿态变化率（Attitude Derivative）
    vec3 body_rate_dot_radps{};  // 角速度变化率，单位 rad/s²（Angular Rate Derivative in rad/s²）
    double mass_dot_kgps = 0.0;  // 质量变化率，单位 kg/s（Mass Derivative in kg/s）
};

// 世界坐标系重力加速度向量（Gravity Acceleration Vector in WCS）
inline vec3 gravity_wcs() {
    return {0.0, 0.0, constants::gravity_mps2};
}

// 将机体坐标系向量转换到世界坐标系（Convert Body-Frame Vector to WCS）
inline vec3 body_to_wcs(const rigid_body_state& state, const vec3& body_vec) {
    return state.attitude_body_to_wcs.rotate(body_vec);
}

// 将世界坐标系向量转换到机体坐标系（Convert WCS Vector to Body Frame）
inline vec3 wcs_to_body(const rigid_body_state& state, const vec3& wcs_vec) {
    return state.attitude_body_to_wcs.inverse().rotate(wcs_vec);
}

// 计算刚体动力学导数（Evaluate Rigid Body Dynamics Derivative）
inline rigid_body_derivative evaluate_rigid_body_dynamics(const rigid_body_state& state,
                                                          const rigid_body_inputs& input) {
    rigid_body_derivative out;
    out.position_dot_wcs = state.velocity_wcs;

    double mass = std::max(state.mass_kg, 1.0e-6);
    vec3 force_wcs = body_to_wcs(state, input.force_body_n);
    out.velocity_dot_wcs = force_wcs / mass + gravity_wcs() + input.external_accel_wcs;

    double ix = std::max(state.inertia_diag_kgm2.x, 1.0e-6);
    double iy = std::max(state.inertia_diag_kgm2.y, 1.0e-6);
    double iz = std::max(state.inertia_diag_kgm2.z, 1.0e-6);
    vec3 inertia_omega{ix * state.body_rates_radps.x,
                       iy * state.body_rates_radps.y,
                       iz * state.body_rates_radps.z};
    vec3 net_moment = input.moment_body_nm - state.body_rates_radps.cross(inertia_omega);
    out.body_rate_dot_radps = {net_moment.x / ix, net_moment.y / iy, net_moment.z / iz};

    quaternion omega_q{0.0,
                       state.body_rates_radps.x,
                       state.body_rates_radps.y,
                       state.body_rates_radps.z};
    out.attitude_dot = (state.attitude_body_to_wcs * omega_q) * 0.5;
    out.mass_dot_kgps = -std::max(input.mass_flow_kgps, 0.0);
    return out;
}

// 六自由度积分器（6-DOF Integrator）
struct six_dof_integrator {
    // 推进刚体状态一步（Step Rigid Body State Forward by One Time Step）
    void step(rigid_body_state& state,
              const rigid_body_inputs& input,
              double dt_s) const {
        if (dt_s <= 0.0) return;
        rigid_body_derivative dx = evaluate_rigid_body_dynamics(state, input);
        state.position_wcs += dx.position_dot_wcs * dt_s;
        state.velocity_wcs += dx.velocity_dot_wcs * dt_s;
        state.body_rates_radps += dx.body_rate_dot_radps * dt_s;
        state.attitude_body_to_wcs =
            (state.attitude_body_to_wcs + dx.attitude_dot * dt_s).normalized();
        state.mass_kg = std::max(state.mass_kg + dx.mass_dot_kgps * dt_s, 1.0e-6);
    }
};

}  // namespace xsf_math
