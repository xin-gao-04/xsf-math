#pragma once

#include "../core/constants.hpp"
#include "../core/coordinate_transform.hpp"
#include "proportional_nav.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace xsf_math {

struct seeker_measurement {
    bool locked = false;
    bool in_fov = false;
    bool reacquired = false;
    double azimuth_rad = 0.0;
    double elevation_rad = 0.0;
    double line_of_sight_rate_rad_s = 0.0;
    vec3 los_unit_wcs{};
};

struct seeker_state {
    bool has_lock = false;
    double last_seen_time_s = -1.0;
};

struct seeker {
    double azimuth_fov_rad = 25.0 * constants::deg_to_rad;
    double elevation_fov_rad = 25.0 * constants::deg_to_rad;
    double reacquire_delay_s = 0.5;
    double angle_noise_std_rad = 0.1 * constants::deg_to_rad;
    bool stochastic = false;
    mutable std::mt19937 rng{7};

    seeker_measurement observe(const engagement_geometry& geom,
                               const euler_angles& weapon_attitude,
                               seeker_state& state,
                               double sim_time_s) const {
        seeker_measurement out;
        vec3 los_wcs = geom.relative_pos().normalized();
        vec3 los_ecs = wcs_to_ecs(los_wcs, weapon_attitude);

        double az = std::atan2(los_ecs.y, los_ecs.x);
        double el = std::atan2(-los_ecs.z, std::max(std::sqrt(los_ecs.x * los_ecs.x + los_ecs.y * los_ecs.y), 1.0e-12));

        if (stochastic) {
            std::normal_distribution<double> noise(0.0, angle_noise_std_rad);
            az += noise(rng);
            el += noise(rng);
        }

        out.in_fov = std::abs(az) <= azimuth_fov_rad && std::abs(el) <= elevation_fov_rad;
        if (out.in_fov) {
            bool reacquired = !state.has_lock && state.last_seen_time_s >= 0.0 &&
                              (sim_time_s - state.last_seen_time_s) >= reacquire_delay_s;
            state.has_lock = true;
            state.last_seen_time_s = sim_time_s;
            out.locked = true;
            out.reacquired = reacquired;
        } else {
            state.has_lock = false;
        }

        out.azimuth_rad = az;
        out.elevation_rad = el;
        out.line_of_sight_rate_rad_s = geom.los_rate().magnitude();
        out.los_unit_wcs = los_wcs;
        return out;
    }
};

}  // namespace xsf_math
