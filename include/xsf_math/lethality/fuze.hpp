#pragma once

#include "../core/vec3.hpp"
#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Proximity fuze and lethality models

// Fuze detonation result
enum class fuze_result {
    no_trigger,       // fuze not triggered (still searching)
    proximity_burst,  // proximity detonation
    contact,          // direct hit
    miss,             // target passed, fuze expired
    dud               // fuze malfunction
};

// Closest point of approach (CPA) computation
struct cpa_result {
    double miss_distance_m = 1e10;  // closest approach distance
    double time_to_cpa_s   = 0.0;  // time until CPA (negative = already passed)
    vec3   cpa_point_weapon;        // weapon position at CPA
    vec3   cpa_point_target;        // target position at CPA
};

// Compute CPA assuming constant velocity for both weapon and target
inline cpa_result compute_cpa(const vec3& weapon_pos, const vec3& weapon_vel,
                               const vec3& target_pos, const vec3& target_vel) {
    vec3 dr = target_pos - weapon_pos;
    vec3 dv = target_vel - weapon_vel;
    double dv2 = dv.magnitude_sq();

    cpa_result r;
    if (dv2 < 1e-20) {
        // No relative motion
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

// Proximity fuze model
struct proximity_fuze {
    double trigger_radius_m = 10.0;    // proximity trigger distance
    double arm_delay_s      = 1.0;     // arming delay after launch
    double arm_range_m      = 500.0;   // minimum range before arming
    double dud_probability  = 0.0;     // probability of dud [0-1]

    // Check fuze trigger condition
    fuze_result check(double flight_time_s,
                      const vec3& weapon_pos, const vec3& weapon_vel,
                      const vec3& target_pos, const vec3& target_vel,
                      double dt_s) const {

        // Check arming conditions
        if (flight_time_s < arm_delay_s) return fuze_result::no_trigger;

        double range = (target_pos - weapon_pos).magnitude();
        // Note: arm_range_m is minimum range from launch, not from target

        // CPA check
        auto cpa = compute_cpa(weapon_pos, weapon_vel, target_pos, target_vel);

        // Contact check (very close)
        if (range < 1.0) return fuze_result::contact;

        // Proximity trigger
        if (range <= trigger_radius_m && cpa.time_to_cpa_s <= dt_s) {
            return fuze_result::proximity_burst;
        }

        // Check if target has been passed (CPA in the past)
        if (cpa.time_to_cpa_s < -dt_s && cpa.miss_distance_m > trigger_radius_m) {
            return fuze_result::miss;
        }

        return fuze_result::no_trigger;
    }
};

// Two-stage PCA (Proximity Check Algorithm)
// Stage 1: Coarse gate (large radius, fast check)
// Stage 2: Fine gate (precise distance computation)
struct pca_two_stage {
    double coarse_gate_m = 100.0;  // stage 1 radius
    double fine_gate_m   = 10.0;   // stage 2 radius (trigger)

    struct result {
        bool   in_coarse_gate = false;
        bool   in_fine_gate   = false;
        double distance_m     = 1e10;
    };

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
