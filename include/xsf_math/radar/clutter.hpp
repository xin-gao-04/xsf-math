#pragma once

#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 地表杂波模型

// 地杂波单位面积 RCS（sigma_0）模型
enum class surface_type {
    sea_calm,
    sea_moderate,
    sea_rough,
    rural,
    suburban,
    urban,
    desert,
    forest
};

// 恒定 gamma 杂波模型
// sigma_0 = gamma * sin(grazing_angle)
struct constant_gamma_clutter {
    double gamma_db = -20.0;  // 杂波系数（dB）

    double sigma0(double grazing_angle_rad) const {
        double gamma = db_to_linear(gamma_db);
        return gamma * std::sin(std::abs(grazing_angle_rad));
    }

    // 根据地表类型获取典型 gamma
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

// 杂波面积计算
struct clutter_geometry {
    double range_m;
    double grazing_angle_rad;
    double azimuth_beamwidth_rad;
    double pulse_width_s;
    double range_resolution_m;  // c * tau / 2

    // 脉冲限制雷达的杂波斑块面积
    double patch_area_pulse() const {
        return range_m * azimuth_beamwidth_rad * range_resolution_m;
    }

    // 波束限制雷达的杂波斑块面积
    double patch_area_beam(double elevation_beamwidth_rad) const {
        return range_m * range_m * azimuth_beamwidth_rad * elevation_beamwidth_rad;
    }
};

// 雷达接收的杂波功率
inline double clutter_power_w(
        double tx_power_w,
        double tx_gain_linear,
        double rx_gain_linear,
        double frequency_hz,
        double range_m,
        double sigma0,           // 单位面积杂波 RCS
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

// MTI 改善因子（简化）
// 运动目标显示处理后的杂波功率抑制
inline double mti_improvement_factor_db(int num_pulses, double clutter_velocity_spread_mps,
                                         double prf_hz, double frequency_hz) {
    // MTI 改善近似为：
    // I = N^2 * (PRF / (2 * sigma_v / lambda))^2（单消除器）
    double lambda = constants::speed_of_light / frequency_hz;
    if (clutter_velocity_spread_mps < 1e-10) return 60.0;  // 接近静止的杂波

    double sigma_f = clutter_velocity_spread_mps / lambda;  // 频谱展宽（Hz）
    double ratio = prf_hz / (2.0 * sigma_f);

    double improvement = static_cast<double>(num_pulses * num_pulses) * ratio * ratio;
    return std::min(linear_to_db(improvement), 60.0);  // 封顶 60 dB
}

// 脉冲多普勒杂波抑制
inline double pd_clutter_rejection_db(double mti_db, double filter_rejection_db = 30.0) {
    return mti_db + filter_rejection_db;
}

} // namespace xsf_math
