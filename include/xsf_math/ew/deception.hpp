#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct rgpo_profile {
    double pull_off_rate_mps = 150.0;
    double max_range_offset_m = 3000.0;

    double range_offset(double time_since_start_s) const {
        return std::clamp(time_since_start_s * pull_off_rate_mps, 0.0, max_range_offset_m);
    }
};

struct vgpo_profile {
    double pull_off_accel_mps2 = 25.0;
    double max_velocity_offset_mps = 300.0;

    double velocity_offset(double time_since_start_s) const {
        return std::clamp(time_since_start_s * pull_off_accel_mps2, 0.0, max_velocity_offset_mps);
    }
};

struct angle_deception_profile {
    double azimuth_bias_rad = 0.0;
    double elevation_bias_rad = 0.0;
};

struct deceptive_target_state {
    double apparent_range_offset_m = 0.0;
    double apparent_velocity_offset_mps = 0.0;
    double apparent_azimuth_bias_rad = 0.0;
    double apparent_elevation_bias_rad = 0.0;
};

inline deceptive_target_state apply_rgpo(const rgpo_profile& rgpo, double time_since_start_s) {
    deceptive_target_state out;
    out.apparent_range_offset_m = rgpo.range_offset(time_since_start_s);
    return out;
}

inline deceptive_target_state apply_vgpo(const vgpo_profile& vgpo, double time_since_start_s) {
    deceptive_target_state out;
    out.apparent_velocity_offset_mps = vgpo.velocity_offset(time_since_start_s);
    return out;
}

inline deceptive_target_state apply_angle_deception(const angle_deception_profile& angle) {
    deceptive_target_state out;
    out.apparent_azimuth_bias_rad = angle.azimuth_bias_rad;
    out.apparent_elevation_bias_rad = angle.elevation_bias_rad;
    return out;
}

struct rwr_contact {
    bool detected = false;
    bool tracked = false;
    double pulse_density = 0.0;
    double threat_score = 0.0;
    double range_estimate_m = 0.0;
};

struct rwr_receiver {
    double sensitivity_w = 1.0e-12;
    double track_threshold_w = 5.0e-12;

    rwr_contact evaluate(double incident_power_w, double radar_prf_hz, double ownship_range_m) const {
        rwr_contact out;
        out.detected = incident_power_w >= sensitivity_w;
        out.tracked = incident_power_w >= track_threshold_w;
        out.pulse_density = radar_prf_hz;
        out.range_estimate_m = ownship_range_m;
        if (out.detected) {
            double power_ratio = incident_power_w / std::max(sensitivity_w, 1.0e-20);
            out.threat_score = std::log1p(power_ratio) * (out.tracked ? 2.0 : 1.0);
        }
        return out;
    }
};

}  // namespace xsf_math
