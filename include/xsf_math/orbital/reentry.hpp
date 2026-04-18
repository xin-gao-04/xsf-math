#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct reentry_state {
    double altitude_m = 120000.0;
    double downrange_m = 0.0;
    double velocity_mps = 7500.0;
    double flight_path_angle_rad = -5.0 * constants::deg_to_rad;
    double mass_kg = 1000.0;
    double heat_load_j_cm2 = 0.0;
};

struct reentry_vehicle {
    double reference_area_m2 = 1.0;
    double drag_coefficient = 1.2;
    double lift_to_drag = 0.0;
    double ballistic_coeff_kg_m2 = 300.0;
};

struct reentry_environment {
    double sea_level_density_kg_m3 = constants::ssl_density;
    double scale_height_m = 7200.0;
    double nose_radius_m = 0.5;
};

struct reentry_step_result {
    double density_kg_m3 = 0.0;
    double dynamic_pressure_pa = 0.0;
    double convective_heat_flux_w_m2 = 0.0;
    double deceleration_mps2 = 0.0;
};

struct reentry_propagator {
    reentry_vehicle vehicle{};
    reentry_environment environment{};

    double atmosphere_density(double altitude_m) const {
        return environment.sea_level_density_kg_m3 *
               std::exp(-std::max(altitude_m, 0.0) / std::max(environment.scale_height_m, 1.0));
    }

    reentry_step_result step(reentry_state& state, double dt_s) const {
        reentry_step_result out;
        if (dt_s <= 0.0) return out;

        double radius = constants::earth_radius_m + std::max(state.altitude_m, 0.0);
        double g = constants::gravity_mps2 * (constants::earth_radius_m * constants::earth_radius_m) / (radius * radius);
        out.density_kg_m3 = atmosphere_density(state.altitude_m);
        out.dynamic_pressure_pa = 0.5 * out.density_kg_m3 * state.velocity_mps * state.velocity_mps;

        double drag = out.dynamic_pressure_pa * vehicle.reference_area_m2 * vehicle.drag_coefficient;
        double lift = drag * vehicle.lift_to_drag;
        double vdot = -(drag / std::max(state.mass_kg, 1.0)) - g * std::sin(state.flight_path_angle_rad);
        double gamma_dot = lift / std::max(state.mass_kg * std::max(state.velocity_mps, 1.0), 1.0) +
                           (state.velocity_mps / radius - g / std::max(state.velocity_mps, 1.0)) *
                               std::cos(state.flight_path_angle_rad);
        double hdot = state.velocity_mps * std::sin(state.flight_path_angle_rad);
        double xdot = state.velocity_mps * std::cos(state.flight_path_angle_rad);

        state.velocity_mps = std::max(state.velocity_mps + vdot * dt_s, 0.0);
        state.flight_path_angle_rad += gamma_dot * dt_s;
        state.altitude_m = std::max(state.altitude_m + hdot * dt_s, 0.0);
        state.downrange_m += xdot * dt_s;

        out.convective_heat_flux_w_m2 =
            1.83e-4 * std::sqrt(out.density_kg_m3 / std::max(environment.nose_radius_m, 1.0e-3)) *
            std::pow(state.velocity_mps, 3.0);
        state.heat_load_j_cm2 += out.convective_heat_flux_w_m2 * dt_s / 1.0e4;
        out.deceleration_mps2 = -vdot;
        return out;
    }
};

}  // namespace xsf_math
