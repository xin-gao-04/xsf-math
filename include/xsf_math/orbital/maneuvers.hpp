#pragma once

#include "kepler.hpp"
#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct hohmann_transfer_result {
    double transfer_semi_major_axis_m = 0.0;
    double delta_v1_mps               = 0.0;
    double delta_v2_mps               = 0.0;
    double total_delta_v_mps          = 0.0;
    double transfer_time_s            = 0.0;
};

inline hohmann_transfer_result hohmann_transfer(double initial_radius_m,
                                                double final_radius_m,
                                                double gravitational_parameter = mu_earth) {
    hohmann_transfer_result result;
    if (initial_radius_m <= 0.0 || final_radius_m <= 0.0) return result;

    double a_transfer = 0.5 * (initial_radius_m + final_radius_m);
    double v1         = std::sqrt(gravitational_parameter / initial_radius_m);
    double v2         = std::sqrt(gravitational_parameter / final_radius_m);
    double vt1        = std::sqrt(gravitational_parameter * (2.0 / initial_radius_m - 1.0 / a_transfer));
    double vt2        = std::sqrt(gravitational_parameter * (2.0 / final_radius_m - 1.0 / a_transfer));

    result.transfer_semi_major_axis_m = a_transfer;
    result.delta_v1_mps               = vt1 - v1;
    result.delta_v2_mps               = v2 - vt2;
    result.total_delta_v_mps          = std::abs(result.delta_v1_mps) + std::abs(result.delta_v2_mps);
    result.transfer_time_s            = constants::pi * std::sqrt(std::pow(a_transfer, 3) / gravitational_parameter);
    return result;
}

inline double circularize_delta_v(double radius_m,
                                  double current_speed_mps,
                                  double gravitational_parameter = mu_earth) {
    if (radius_m <= 0.0) return 0.0;
    double circular_speed = std::sqrt(gravitational_parameter / radius_m);
    return circular_speed - current_speed_mps;
}

inline double circularize_delta_v(const vec3& position_eci,
                                  const vec3& velocity_eci,
                                  double gravitational_parameter = mu_earth) {
    double radius = position_eci.magnitude();
    return circularize_delta_v(radius, velocity_eci.magnitude(), gravitational_parameter);
}

inline double plane_change_delta_v(double speed_mps, double plane_change_angle_rad) {
    return 2.0 * speed_mps * std::sin(0.5 * std::abs(plane_change_angle_rad));
}

inline vec3 orbital_plane_normal(double inclination_rad, double raan_rad) {
    double sin_i = std::sin(inclination_rad);
    double cos_i = std::cos(inclination_rad);
    double sin_w = std::sin(raan_rad);
    double cos_w = std::cos(raan_rad);
    return {sin_i * sin_w, -sin_i * cos_w, cos_i};
}

inline double plane_change_angle(double inclination_initial_rad,
                                 double raan_initial_rad,
                                 double inclination_final_rad,
                                 double raan_final_rad) {
    vec3 n0 = orbital_plane_normal(inclination_initial_rad, raan_initial_rad).normalized();
    vec3 n1 = orbital_plane_normal(inclination_final_rad, raan_final_rad).normalized();
    return std::acos(std::clamp(n0.dot(n1), -1.0, 1.0));
}

inline double inclination_change_delta_v(double speed_mps,
                                         double inclination_initial_rad,
                                         double inclination_final_rad) {
    return plane_change_delta_v(speed_mps, inclination_final_rad - inclination_initial_rad);
}

inline double raan_inclination_change_delta_v(double speed_mps,
                                              double inclination_initial_rad,
                                              double raan_initial_rad,
                                              double inclination_final_rad,
                                              double raan_final_rad) {
    double angle = plane_change_angle(inclination_initial_rad,
                                      raan_initial_rad,
                                      inclination_final_rad,
                                      raan_final_rad);
    return plane_change_delta_v(speed_mps, angle);
}

} // namespace xsf_math
