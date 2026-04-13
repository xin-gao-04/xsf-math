#pragma once

#include "kepler.hpp"
#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 地球 J2 摄动常数
constexpr double j2_earth = 0.0010826267;

struct j2_params {
    double gravitational_parameter = mu_earth;
    double mean_radius_m           = constants::earth_radius_m;
    double j2                      = j2_earth;
};

struct j2_secular_rates {
    double raan_rate_rad_s          = 0.0;
    double arg_periapsis_rate_rad_s = 0.0;
    double mean_anomaly_rate_rad_s  = 0.0;
};

inline double normalize_angle_0_2pi(double angle_rad) {
    double wrapped = std::fmod(angle_rad, constants::two_pi);
    if (wrapped < 0.0) wrapped += constants::two_pi;
    return wrapped;
}

// 在惯性系中计算 J2 摄动加速度。
inline vec3 j2_acceleration(const vec3& position_eci, const j2_params& params = {}) {
    double r2 = position_eci.magnitude_sq();
    if (r2 < 1.0e-20) return {};

    double z2    = position_eci.z * position_eci.z;
    double r     = std::sqrt(r2);
    double r4    = r2 * r2;
    double coeff = 1.5 * params.gravitational_parameter * params.mean_radius_m * params.mean_radius_m * params.j2 /
                   (r4 * r);
    double term = 5.0 * z2 / r2;

    return {
        position_eci.x * coeff * (term - 1.0),
        position_eci.y * coeff * (term - 1.0),
        position_eci.z * coeff * (term - 3.0)
    };
}

// J2 一阶长期漂移率。
inline j2_secular_rates compute_j2_secular_rates(const orbital_elements& oe, const j2_params& params = {}) {
    j2_secular_rates rates;

    if (oe.semi_major_axis_m <= 0.0 || oe.eccentricity >= 1.0) {
        return rates;
    }

    double e2 = oe.eccentricity * oe.eccentricity;
    double p  = oe.semi_major_axis_m * (1.0 - e2);
    if (p <= 0.0) return rates;

    double n       = std::sqrt(params.gravitational_parameter / std::pow(oe.semi_major_axis_m, 3));
    double factor  = params.j2 * std::pow(params.mean_radius_m / p, 2);
    double cos_i   = std::cos(oe.inclination_rad);
    double cos_i_2 = cos_i * cos_i;
    double root    = std::sqrt(std::max(0.0, 1.0 - e2));

    rates.raan_rate_rad_s = -1.5 * n * factor * cos_i;
    rates.arg_periapsis_rate_rad_s = 0.75 * n * factor * (5.0 * cos_i_2 - 1.0);
    rates.mean_anomaly_rate_rad_s = n + 0.75 * n * factor * root * (3.0 * cos_i_2 - 1.0);
    return rates;
}

// 在二体开普勒推进的基础上加入 J2 长期岁差。
inline orbital_elements propagate_j2_secular(const orbital_elements& oe,
                                             double dt_s,
                                             const j2_params& params = {}) {
    orbital_elements result = propagate_kepler(oe, dt_s);
    j2_secular_rates rates  = compute_j2_secular_rates(oe, params);

    result.raan_rad = normalize_angle_0_2pi(oe.raan_rad + rates.raan_rate_rad_s * dt_s);
    result.arg_periapsis_rad =
        normalize_angle_0_2pi(oe.arg_periapsis_rad + rates.arg_periapsis_rate_rad_s * dt_s);

    double E0 = true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity);
    double M0 = eccentric_to_mean_anomaly(E0, oe.eccentricity);
    double M  = normalize_angle_0_2pi(M0 + rates.mean_anomaly_rate_rad_s * dt_s);
    double E  = solve_kepler(M, oe.eccentricity);
    result.true_anomaly_rad = normalize_angle_0_2pi(eccentric_to_true_anomaly(E, oe.eccentricity));
    return result;
}

} // namespace xsf_math
