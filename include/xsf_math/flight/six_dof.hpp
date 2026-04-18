#pragma once

#include "../core/constants.hpp"
#include "../core/quaternion.hpp"
#include "../core/vec3.hpp"
#include <algorithm>

namespace xsf_math {

struct rigid_body_inputs {
    vec3 force_body_n{};
    vec3 moment_body_nm{};
    vec3 external_accel_wcs{};
    double mass_flow_kgps = 0.0;
};

struct rigid_body_state {
    vec3 position_wcs{};
    vec3 velocity_wcs{};
    quaternion attitude_body_to_wcs{};
    vec3 body_rates_radps{};
    double mass_kg = 1.0;
    vec3 inertia_diag_kgm2{1.0, 1.0, 1.0};

    vec3 body_velocity_mps() const {
        return attitude_body_to_wcs.inverse().rotate(velocity_wcs);
    }
};

struct rigid_body_derivative {
    vec3 position_dot_wcs{};
    vec3 velocity_dot_wcs{};
    quaternion attitude_dot{};
    vec3 body_rate_dot_radps{};
    double mass_dot_kgps = 0.0;
};

inline vec3 gravity_wcs() {
    return {0.0, 0.0, constants::gravity_mps2};
}

inline vec3 body_to_wcs(const rigid_body_state& state, const vec3& body_vec) {
    return state.attitude_body_to_wcs.rotate(body_vec);
}

inline vec3 wcs_to_body(const rigid_body_state& state, const vec3& wcs_vec) {
    return state.attitude_body_to_wcs.inverse().rotate(wcs_vec);
}

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

struct six_dof_integrator {
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
