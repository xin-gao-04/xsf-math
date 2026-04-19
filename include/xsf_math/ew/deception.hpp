#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 距离拖引（Range Gate Pull Off, RGPO）参数剖面 (RGPO profile parameters)
struct rgpo_profile {
    double pull_off_rate_mps = 150.0;    // 拖引速率（Pull-off rate），单位 m/s
    double max_range_offset_m = 3000.0;  // 最大距离偏置（Maximum range offset），单位 m

    // 计算给定时刻的距离偏置（Calculate range offset at given time）
    double range_offset(double time_since_start_s) const {
        return std::clamp(time_since_start_s * pull_off_rate_mps, 0.0, max_range_offset_m);
    }
};

// 速度拖引（Velocity Gate Pull Off, VGPO）参数剖面 (VGPO profile parameters)
struct vgpo_profile {
    double pull_off_accel_mps2 = 25.0;     // 拖引加速度（Pull-off acceleration），单位 m/s^2
    double max_velocity_offset_mps = 300.0;// 最大速度偏置（Maximum velocity offset），单位 m/s

    // 计算给定时刻的速度偏置（Calculate velocity offset at given time）
    double velocity_offset(double time_since_start_s) const {
        return std::clamp(time_since_start_s * pull_off_accel_mps2, 0.0, max_velocity_offset_mps);
    }
};

// 角度欺骗参数剖面（Angle deception profile）
struct angle_deception_profile {
    double azimuth_bias_rad = 0.0;    // 方位角偏置（Azimuth bias），单位 rad
    double elevation_bias_rad = 0.0;  // 俯仰角偏置（Elevation bias），单位 rad
};

// 欺骗性目标状态（Deceptive target state）
struct deceptive_target_state {
    double apparent_range_offset_m = 0.0;       // 视在距离偏置（Apparent range offset），单位 m
    double apparent_velocity_offset_mps = 0.0;  // 视在速度偏置（Apparent velocity offset），单位 m/s
    double apparent_azimuth_bias_rad = 0.0;     // 视在方位角偏置（Apparent azimuth bias），单位 rad
    double apparent_elevation_bias_rad = 0.0;   // 视在俯仰角偏置（Apparent elevation bias），单位 rad
};

// 应用距离拖引（Apply RGPO deception）
inline deceptive_target_state apply_rgpo(const rgpo_profile& rgpo, double time_since_start_s) {
    deceptive_target_state out;
    out.apparent_range_offset_m = rgpo.range_offset(time_since_start_s);
    return out;
}

// 应用速度拖引（Apply VGPO deception）
inline deceptive_target_state apply_vgpo(const vgpo_profile& vgpo, double time_since_start_s) {
    deceptive_target_state out;
    out.apparent_velocity_offset_mps = vgpo.velocity_offset(time_since_start_s);
    return out;
}

// 应用角度欺骗（Apply angle deception）
inline deceptive_target_state apply_angle_deception(const angle_deception_profile& angle) {
    deceptive_target_state out;
    out.apparent_azimuth_bias_rad = angle.azimuth_bias_rad;
    out.apparent_elevation_bias_rad = angle.elevation_bias_rad;
    return out;
}

// 雷达告警接收机（Radar Warning Receiver, RWR）接触信息 (RWR contact information)
struct rwr_contact {
    bool detected = false;            // 是否被探测到（Detected）
    bool tracked = false;             // 是否被跟踪（Tracked）
    double pulse_density = 0.0;       // 脉冲密度（Pulse density），单位 Hz
    double threat_score = 0.0;        // 威胁等级评分（Threat score）
    double range_estimate_m = 0.0;    // 距离估计（Range estimate），单位 m
};

// 雷达告警接收机模型（RWR receiver model）
struct rwr_receiver {
    double sensitivity_w = 1.0e-12;      // 接收灵敏度（Sensitivity），单位 W
    double track_threshold_w = 5.0e-12;  // 跟踪门限（Track threshold），单位 W

    // 评估入射信号（Evaluate incident signal）
    // incident_power_w: 入射功率（W）; radar_prf_hz: 雷达脉冲重复频率（Hz）; ownship_range_m: 本机距离（m）
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
