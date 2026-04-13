#pragma once

#include "../core/vec3.hpp"
#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Relative geometry between weapon and target
struct engagement_geometry {
    vec3   weapon_pos;       // weapon position (WCS)
    vec3   weapon_vel;       // weapon velocity (WCS)
    vec3   target_pos;       // target position (WCS)
    vec3   target_vel;       // target velocity (WCS)
    vec3   target_accel;     // target acceleration estimate (WCS), for APN

    // Derived quantities
    vec3 relative_pos() const { return target_pos - weapon_pos; }
    vec3 relative_vel() const { return target_vel - weapon_vel; }

    double slant_range() const { return relative_pos().magnitude(); }

    // Closing velocity (positive when approaching)
    double closing_velocity() const {
        vec3 rp = relative_pos();
        double R = rp.magnitude();
        if (R < 1e-10) return 0.0;
        return -relative_vel().dot(rp) / R;
    }

    // LOS unit vector
    vec3 los_unit() const { return relative_pos().normalized(); }

    // LOS angular rate vector: omega = (R x V_rel) / |R|^2
    vec3 los_rate() const {
        vec3 R = relative_pos();
        double R2 = R.magnitude_sq();
        if (R2 < 1e-20) return {0, 0, 0};
        return R.cross(relative_vel()) / R2;
    }

    // Time to intercept estimate (assuming constant closing velocity)
    double time_to_intercept() const {
        double vc = closing_velocity();
        if (vc <= 0.0) return 1e10;  // diverging
        return slant_range() / vc;
    }

    // Ground range (horizontal distance)
    double ground_range() const {
        vec3 r = relative_pos();
        return std::sqrt(r.x*r.x + r.y*r.y);
    }

    // LOS azimuth and elevation
    double los_azimuth() const {
        vec3 r = relative_pos();
        return std::atan2(r.y, r.x);
    }

    double los_elevation() const {
        vec3 r = relative_pos();
        double gr = std::sqrt(r.x*r.x + r.y*r.y);
        return std::atan2(-r.z, gr);  // Z down
    }
};

// Pure Proportional Navigation (PN)
// a_cmd = N * V_c * dtheta/dt  (scalar form)
// a_cmd = N * (omega x V_wpn)   (3D vector form)
struct proportional_nav {
    double nav_ratio = 4.0;  // N (typical 3-5)

    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 omega = geom.los_rate();
        vec3 accel = nav_ratio * omega.cross(geom.weapon_vel);
        return accel;
    }
};

// Augmented Proportional Navigation (APN)
// a_cmd = N * (omega x V_wpn) + (N/2) * a_target
struct augmented_proportional_nav {
    double nav_ratio = 4.0;

    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 omega = geom.los_rate();
        vec3 pn_accel = nav_ratio * omega.cross(geom.weapon_vel);
        vec3 aug_term = (nav_ratio / 2.0) * geom.target_accel;
        return pn_accel + aug_term;
    }
};

// Pursuit guidance
// Steers velocity vector toward target current position
// a_cmd = K * sin(target_offset_angle) * g
struct pursuit_guidance {
    double gain = 3.0;  // K (in g's per radian, typical 2-5)

    vec3 compute_accel(const engagement_geometry& geom) const {
        vec3 aim_dir = geom.relative_pos().normalized();
        vec3 vel_dir = geom.weapon_vel.normalized();
        double speed = geom.weapon_vel.magnitude();

        if (speed < 1e-10) return {0, 0, 0};

        // Double cross product method (avoids explicit trig):
        // n = V x AimDir  -> normal to maneuvering plane
        // lateral = n x V -> lateral direction in plane
        // |lateral| / |V|^2 = sin(offset_angle)
        vec3 n = vel_dir.cross(aim_dir);
        vec3 lateral = n.cross(vel_dir);

        double lateral_mag = lateral.magnitude();
        // lateral_mag = sin(offset_angle) when both inputs are unit vectors

        if (lateral_mag < 1e-10) return {0, 0, 0};

        vec3 accel = lateral.normalized() * (gain * constants::gravity_mps2 * lateral_mag);
        return accel;
    }
};

// Guidance acceleration limiter
// Constrains command to available aerodynamic capability
struct accel_limiter {
    double max_g = 30.0;  // maximum available g-load

    vec3 limit(const vec3& accel_cmd) const {
        double max_accel = max_g * constants::gravity_mps2;
        double mag = accel_cmd.magnitude();
        if (mag <= max_accel) return accel_cmd;
        return accel_cmd * (max_accel / mag);
    }

    // Compute max available g given dynamic pressure and missile params
    static double max_available_g(double dynamic_pressure_pa,
                                   double ref_area_m2,
                                   double cl_max,
                                   double mass_kg) {
        if (mass_kg <= 0.0) return 0.0;
        double max_force = dynamic_pressure_pa * ref_area_m2 * cl_max;
        return max_force / (mass_kg * constants::gravity_mps2);
    }
};

// Multi-phase guidance state machine
enum class guidance_phase_trigger {
    flight_time,
    altitude,
    speed,
    mach,
    dynamic_pressure,
    target_range,
    ground_range,
    time_to_intercept,
    los_elevation,
    los_azimuth
};

struct phase_transition {
    guidance_phase_trigger trigger;
    double threshold;
    bool   trigger_above = true;  // true: trigger when value > threshold
};

// Evaluate whether a phase transition should occur
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
