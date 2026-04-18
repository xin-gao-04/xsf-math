#pragma once

#include "../core/constants.hpp"
#include "../core/rcs.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct sphere_rcs {
    double radius_m = 1.0;

    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double size_ratio = radius_m / lambda;
        if (size_ratio < 0.2) {
            double sigma = 9.0 * constants::pi * std::pow(radius_m, 6) / std::pow(lambda, 4);
            return rcs_value::from_m2(std::max(sigma, 0.0));
        }
        return rcs_value::from_m2(constants::pi * radius_m * radius_m);
    }
};

struct flat_plate_rcs {
    double width_m = 1.0;
    double height_m = 1.0;

    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double area = width_m * height_m;
        double cos_inc = std::max(std::cos(az_rad) * std::cos(el_rad), 0.0);
        double sigma = 4.0 * constants::pi * area * area * cos_inc * cos_inc / (lambda * lambda);
        return rcs_value::from_m2(std::max(sigma, 0.0));
    }
};

struct cylinder_rcs {
    double radius_m = 0.5;
    double length_m = 4.0;

    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double broadside = 2.0 * constants::pi * radius_m * length_m * length_m / std::max(lambda, 1.0e-9);
        double aspect = std::sin(az_rad) * std::sin(az_rad) * std::cos(el_rad) * std::cos(el_rad);
        return rcs_value::from_m2(std::max(broadside * aspect, 0.0));
    }
};

struct corner_reflector_rcs {
    double leg_length_m = 1.0;

    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double sigma = 12.0 * constants::pi * std::pow(leg_length_m, 4) / std::max(lambda * lambda, 1.0e-12);
        return rcs_value::from_m2(std::max(sigma, 0.0));
    }
};

}  // namespace xsf_math
