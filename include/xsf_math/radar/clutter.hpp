#pragma once

#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 地表杂波模型 (Surface Clutter Models)

// 地杂波单位面积 RCS（sigma_0）地表类型 (Surface type for backscatter coefficient sigma_0)
enum class surface_type {
    sea_calm,      // 平静海面 (Calm sea)
    sea_moderate,  // 中等海面 (Moderate sea)
    sea_rough,     // 粗糙海面 (Rough sea)
    rural,         // 乡村 (Rural)
    suburban,      // 郊区 (Suburban)
    urban,         // 城市 (Urban)
    desert,        // 沙漠 (Desert)
    forest         // 森林 (Forest)
};

// 恒定 gamma 杂波模型 (Constant-gamma clutter model)
// sigma_0 = gamma * sin(grazing_angle)
struct constant_gamma_clutter {
    double gamma_db = -20.0;  // 杂波系数（dB）(Clutter coefficient in dB)

    // 计算后向散射系数 sigma_0 (Compute backscatter coefficient sigma_0)
    double sigma0(double grazing_angle_rad) const {
        double gamma = db_to_linear(gamma_db);
        return gamma * std::sin(std::abs(grazing_angle_rad));
    }

    // 根据地表类型获取典型 gamma 值（dB）(Get typical gamma for surface type)
    static double typical_gamma_db(surface_type type) {
        switch (type) {
        case surface_type::sea_calm:      return -50.0;
        case surface_type::sea_moderate:  return -35.0;
        case surface_type::sea_rough:     return -25.0;
        case surface_type::rural:         return -25.0;
        case surface_type::suburban:      return -15.0;
        case surface_type::urban:         return -5.0;
        case surface_type::desert:        return -30.0;
        case surface_type::forest:        return -15.0;
        default: return -20.0;
        }
    }
};

// 杂波面积计算几何参数 (Clutter area geometry parameters)
struct clutter_geometry {
    double range_m;                        // 距离（m）(Range)
    double grazing_angle_rad;              // 擦地角（rad）(Grazing angle)
    double azimuth_beamwidth_rad;          // 方位波束宽度（rad）(Azimuth beamwidth)
    double pulse_width_s;                  // 脉宽（s）(Pulse width)
    double range_resolution_m;             // 距离分辨率：c * tau / 2（m）(Range resolution)

    // 脉冲限制雷达的杂波斑块面积（m^2）(Clutter patch area for pulse-limited radar)
    double patch_area_pulse() const {
        return range_m * azimuth_beamwidth_rad * range_resolution_m;
    }

    // 波束限制雷达的杂波斑块面积（m^2）(Clutter patch area for beam-limited radar)
    double patch_area_beam(double elevation_beamwidth_rad) const {
        return range_m * range_m * azimuth_beamwidth_rad * elevation_beamwidth_rad;
    }
};

// 雷达接收的杂波功率（W）(Radar received clutter power in Watts)
inline double clutter_power_w(
        double tx_power_w,
        double tx_gain_linear,
        double rx_gain_linear,
        double frequency_hz,
        double range_m,
        double sigma0,           // 单位面积杂波 RCS (Clutter RCS per unit area)
        double clutter_area_m2,
        double system_loss_linear = 2.0) {

    double lambda = constants::speed_of_light / frequency_hz;
    double four_pi_cubed = std::pow(4.0 * constants::pi, 3);
    double R4 = std::pow(range_m, 4);

    double numerator = tx_power_w * tx_gain_linear * rx_gain_linear *
                       lambda * lambda * sigma0 * clutter_area_m2;
    double denominator = four_pi_cubed * R4 * system_loss_linear;

    return (denominator > 0.0) ? numerator / denominator : 0.0;
}

// MTI 改善因子（简化）(MTI improvement factor, simplified)
// 运动目标显示处理后的杂波功率抑制 (Clutter suppression after MTI processing)
inline double mti_improvement_factor_db(int num_pulses, double clutter_velocity_spread_mps,
                                         double prf_hz, double frequency_hz) {
    // MTI 改善近似为：
    // I = N^2 * (PRF / (2 * sigma_v / lambda))^2（单消除器）
    double lambda = constants::speed_of_light / frequency_hz;
    if (clutter_velocity_spread_mps < 1e-10) return 60.0;  // 接近静止的杂波 (Near-stationary clutter)

    double sigma_f = clutter_velocity_spread_mps / lambda;  // 频谱展宽（Hz）(Spectral spread)
    double ratio = prf_hz / (2.0 * sigma_f);

    double improvement = static_cast<double>(num_pulses * num_pulses) * ratio * ratio;
    return std::min(linear_to_db(improvement), 60.0);  // 封顶 60 dB (Capped at 60 dB)
}

// 脉冲多普勒杂波抑制（dB）(Pulse-Doppler clutter rejection in dB)
inline double pd_clutter_rejection_db(double mti_db, double filter_rejection_db = 30.0) {
    return mti_db + filter_rejection_db;
}

} // namespace xsf_math
