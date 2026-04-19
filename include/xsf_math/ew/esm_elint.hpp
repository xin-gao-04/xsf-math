#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 辐射源信号参数（Emitter signal parameters）
struct emitter_signal {
    double incident_power_w = 0.0;      // 入射功率（Incident power），单位 W
    double carrier_frequency_hz = 0.0;  // 载波频率（Carrier frequency），单位 Hz
    double prf_hz = 0.0;                // 脉冲重复频率（Pulse Repetition Frequency, PRF），单位 Hz
    double pulse_width_s = 0.0;         // 脉冲宽度（Pulse width），单位 s
    double scan_period_s = 0.0;         // 扫描周期（Scan period），单位 s
};

// 电子支援措施（Electronic Support Measures, ESM）接触信息 (ESM contact information)
struct esm_contact {
    bool intercepted = false;                  // 是否被截获（Intercepted）
    bool classified = false;                   // 是否被分类识别（Classified）
    double intercept_probability = 0.0;        // 截获概率（Intercept probability），范围 [0,1]
    double classification_probability = 0.0;   // 分类概率（Classification probability），范围 [0,1]
    double pulse_density_hz = 0.0;             // 脉冲密度（Pulse density），单位 Hz
    double bearing_accuracy_rad = constants::half_pi;  // 方位精度（Bearing accuracy），单位 rad
};

// 电子支援措施接收机模型（ESM receiver model）
struct esm_receiver {
    double sensitivity_w = 1.0e-12;            // 接收灵敏度（Sensitivity），单位 W
    double classification_threshold_w = 5.0e-12;// 分类识别门限（Classification threshold），单位 W
    int pulses_for_classification = 32;        // 分类所需脉冲数（Pulses required for classification）
    double scan_alignment_factor = 0.8;        // 扫描对准因子（Scan alignment factor），范围 [0,1]

    // 计算截获概率（Calculate intercept probability）
    // signal: 辐射源信号; dwell_time_s: 驻留时间（s）
    double intercept_probability(const emitter_signal& signal, double dwell_time_s) const {
        double power_ratio = signal.incident_power_w / std::max(sensitivity_w, 1.0e-20);
        double pulses = std::max(signal.prf_hz * dwell_time_s, 0.0);
        double pulse_term = 1.0 - std::exp(-pulses / 8.0);
        double power_term = 1.0 - std::exp(-power_ratio);
        return std::clamp(scan_alignment_factor * pulse_term * power_term, 0.0, 1.0);
    }

    // 评估辐射源信号（Evaluate emitter signal）
    // signal: 辐射源信号; dwell_time_s: 驻留时间（s）
    esm_contact evaluate(const emitter_signal& signal, double dwell_time_s) const {
        esm_contact out;
        out.intercept_probability = intercept_probability(signal, dwell_time_s);
        out.intercepted = out.intercept_probability > 0.2;
        out.pulse_density_hz = signal.prf_hz;

        double class_power_ratio =
            signal.incident_power_w / std::max(classification_threshold_w, 1.0e-20);
        double pulses = std::max(signal.prf_hz * dwell_time_s, 0.0);
        double pulse_ratio = pulses / std::max(static_cast<double>(pulses_for_classification), 1.0);
        out.classification_probability =
            std::clamp((1.0 - std::exp(-class_power_ratio)) * std::min(pulse_ratio, 1.0), 0.0, 1.0);
        out.classified = out.classification_probability > 0.5;

        if (out.intercepted) {
            double snr = std::max(signal.incident_power_w / std::max(sensitivity_w, 1.0e-20), 1.0);
            out.bearing_accuracy_rad = std::clamp(0.35 / std::sqrt(snr), 0.5 * constants::deg_to_rad,
                                                  45.0 * constants::deg_to_rad);
        }
        return out;
    }
};

}  // namespace xsf_math
