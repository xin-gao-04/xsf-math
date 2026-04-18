#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct fragment_pattern {
    int fragment_count = 2000;
    double mean_fragment_mass_kg = 0.01;
    double initial_velocity_mps = 1800.0;
    double cone_angle_rad = constants::pi;
};

struct fragment_effect_result {
    double cloud_area_m2 = 0.0;
    double areal_density_per_m2 = 0.0;
    double mean_fragment_energy_j = 0.0;
    double hit_probability = 0.0;
};

inline double gurney_fragment_velocity(double explosive_mass_kg,
                                       double casing_mass_kg,
                                       double gurney_constant_mps = 2400.0) {
    if (explosive_mass_kg <= 0.0 || casing_mass_kg <= 0.0) return 0.0;
    return gurney_constant_mps * std::sqrt(2.0 * explosive_mass_kg / casing_mass_kg);
}

inline fragment_effect_result evaluate_fragment_cloud(const fragment_pattern& pattern,
                                                      double range_m,
                                                      double presented_area_m2) {
    fragment_effect_result out;
    double solid_angle = 2.0 * constants::pi * (1.0 - std::cos(0.5 * pattern.cone_angle_rad));
    out.cloud_area_m2 = std::max(solid_angle * range_m * range_m, 1.0);
    out.areal_density_per_m2 = pattern.fragment_count / out.cloud_area_m2;
    out.mean_fragment_energy_j =
        0.5 * pattern.mean_fragment_mass_kg * pattern.initial_velocity_mps * pattern.initial_velocity_mps;
    out.hit_probability = 1.0 - std::exp(-out.areal_density_per_m2 * std::max(presented_area_m2, 0.0));
    return out;
}

}  // namespace xsf_math
