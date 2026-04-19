#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 声呐环境参数（Sonar environment parameters）
struct sonar_environment {
    double absorption_db_per_km = 0.8;  // 海水吸收损耗（Absorption loss），单位 dB/km
    double ambient_noise_db = 60.0;     // 环境噪声级（Ambient noise level），单位 dB
    double reverberation_db = 55.0;     // 混响级（Reverberation level），单位 dB
};

// 声呐接触信息（Sonar contact information）
struct sonar_contact {
    double transmission_loss_db = 0.0;  // 传播损失（Transmission loss），单位 dB
    double snr_db = 0.0;                // 信噪比（Signal-to-Noise Ratio），单位 dB
    double pd = 0.0;                    // 探测概率（Probability of detection），范围 [0,1]
    bool detected = false;              // 是否被探测到（Detected flag）
};

// 声呐传播损失（Sonar transmission loss, TL）
// range_m: 距离（m）; absorption_db_per_km: 吸收损耗系数（dB/km）
inline double sonar_transmission_loss_db(double range_m, double absorption_db_per_km) {
    double range_km = std::max(range_m / 1000.0, 1.0e-6);
    return 20.0 * std::log10(range_km * 1000.0) + absorption_db_per_km * range_km;
}

// 主动声呐方程（Active sonar equation）
struct active_sonar_equation {
    sonar_environment environment{};        // 声呐环境参数（Sonar environment）
    double source_level_db = 215.0;         // 声源级（Source Level, SL），单位 dB
    double target_strength_db = 10.0;       // 目标强度（Target Strength, TS），单位 dB
    double detection_threshold_db = 10.0;   // 检测阈（Detection Threshold, DT），单位 dB

    // 评估主动声呐探测性能（Evaluate active sonar detection performance）
    // range_m: 目标距离（m）
    sonar_contact evaluate(double range_m) const {
        sonar_contact out;
        out.transmission_loss_db = sonar_transmission_loss_db(range_m, environment.absorption_db_per_km);
        // 主动声呐：回声级 = SL - 2*TL + TS（Echo level）
        double echo_level = source_level_db - 2.0 * out.transmission_loss_db + target_strength_db;
        double interference = std::max(environment.ambient_noise_db, environment.reverberation_db);  // 取环境和混响中较大者
        out.snr_db = echo_level - interference;
        out.pd = std::clamp((out.snr_db - detection_threshold_db + 12.0) / 24.0, 0.0, 1.0);
        out.detected = out.snr_db >= detection_threshold_db;
        return out;
    }
};

// 被动声呐方程（Passive sonar equation）
struct passive_sonar_equation {
    sonar_environment environment{};        // 声呐环境参数（Sonar environment）
    double source_level_db = 165.0;         // 目标辐射噪声源级（Source Level, SL），单位 dB
    double array_gain_db = 12.0;            // 基阵增益（Array Gain, AG），单位 dB
    double detection_threshold_db = 3.0;    // 检测阈（Detection Threshold, DT），单位 dB

    // 评估被动声呐探测性能（Evaluate passive sonar detection performance）
    // range_m: 目标距离（m）
    sonar_contact evaluate(double range_m) const {
        sonar_contact out;
        out.transmission_loss_db = sonar_transmission_loss_db(range_m, environment.absorption_db_per_km);
        // 被动声呐：接收级 = SL - TL + AG（Received level）
        double received_level = source_level_db - out.transmission_loss_db + array_gain_db;
        out.snr_db = received_level - environment.ambient_noise_db;
        out.pd = std::clamp((out.snr_db - detection_threshold_db + 10.0) / 20.0, 0.0, 1.0);
        out.detected = out.snr_db >= detection_threshold_db;
        return out;
    }
};

}  // namespace xsf_math
