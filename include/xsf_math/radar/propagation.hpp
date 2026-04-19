#pragma once

#include "../core/constants.hpp"
#include "../core/atmosphere.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 电磁传播损耗模型 (Electromagnetic Propagation Loss Models)

// 自由空间路径损耗（FSPL），单位 dB (Free-space path loss in dB)
// L_fs = 20*log10(4*pi*R/lambda)
inline double free_space_loss_db(double range_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    if (range_m < 1e-10 || lambda < 1e-30) return 0.0;
    double ratio = 4.0 * constants::pi * range_m / lambda;
    return 20.0 * std::log10(ratio);
}

// 双径（平地）传播因子 (Two-ray ground reflection propagation factor)
// F^4 = 由于地面反射相对于自由空间的增益 (Power factor relative to free space due to ground reflection)
inline double two_ray_factor(double range_m, double h_tx_m, double h_rx_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    double arg = 4.0 * constants::pi * h_tx_m * h_rx_m / (lambda * range_m);
    double sin_val = std::sin(arg);
    double factor = 2.0 * sin_val;  // 电压因子 (Voltage factor)
    return factor * factor;  // 功率因子 F^2 (Power factor)
}

// Blake 大气衰减模型（dB/km，随频率和俯仰角变化）(Blake atmospheric attenuation model)
struct blake_attenuation {
    // 简化模型：衰减率随频率升高而增大，随俯仰角增大而减小（高仰角穿越的大气更少）
    // Simplified model: attenuation increases with frequency, decreases with elevation
    static double rate_db_per_km(double frequency_hz, double elevation_rad) {
        double f_ghz = frequency_hz / 1.0e9;

        // 基础衰减率（晴空、海平面）(Base attenuation rate, clear sea level)
        double alpha;
        if (f_ghz < 1.0) {
            alpha = 0.004;
        } else if (f_ghz < 10.0) {
            alpha = 0.004 + 0.002 * (f_ghz - 1.0);
        } else if (f_ghz < 40.0) {
            // 水汽和氧气吸收峰 (Water vapor and oxygen absorption peaks)
            alpha = 0.022 + 0.01 * std::pow((f_ghz - 10.0) / 30.0, 2.0);
        } else {
            alpha = 0.05;
        }

        // 俯仰角修正（高角度下大气路径更短）(Elevation correction: shorter path at high elevation)
        double el_factor = 1.0;
        if (std::abs(elevation_rad) > 0.01) {
            el_factor = 1.0 / std::sin(std::abs(elevation_rad));
            el_factor = std::min(el_factor, 50.0);  // 在地平线处封顶 (Cap at horizon)
        }

        return alpha * el_factor;
    }

    // 给定距离和频率下的大气总衰减（dB）(Total atmospheric loss for given range and frequency)
    static double total_loss_db(double range_m, double frequency_hz, double elevation_rad = 0.0) {
        return rate_db_per_km(frequency_hz, elevation_rad) * (range_m / 1000.0);
    }
};

// 雨衰减模型（ITU-R P.838）(Rain attenuation model per ITU-R P.838)
struct rain_attenuation {
    double rain_rate_mm_per_hr = 0.0;  // 降雨率（mm/h），0 表示晴空 (Rain rate, 0 = clear sky)

    // 计算雨致损耗（dB）(Compute rain-induced loss in dB)
    double loss_db(double range_m, double frequency_hz) const {
        if (rain_rate_mm_per_hr <= 0.0) return 0.0;

        double f_ghz = frequency_hz / 1.0e9;

        // 简化的 ITU 模型：gamma = k * R^alpha（dB/km）(Simplified ITU model)
        double k, alpha;
        if (f_ghz < 2.0) {
            k = 0.0001; alpha = 1.0;
        } else if (f_ghz < 10.0) {
            k = 0.0001 * std::pow(f_ghz, 2.0);
            alpha = 1.2;
        } else {
            k = 0.01 * std::pow(f_ghz / 10.0, 1.5);
            alpha = 1.1;
        }

        double gamma = k * std::pow(rain_rate_mm_per_hr, alpha);
        return gamma * (range_m / 1000.0);
    }
};

// 透镜损耗（由大气折射导致的波束扩散）(Lens loss due to atmospheric refraction beam spreading)
inline double lens_loss_db(double elevation_rad) {
    // 简化模型：仅在地平线附近显著 (Simplified model: significant only near horizon)
    if (std::abs(elevation_rad) > 10.0 * constants::deg_to_rad)
        return 0.0;
    double el_deg = std::abs(elevation_rad) * constants::rad_to_deg;
    if (el_deg < 0.5) return 1.0;
    if (el_deg < 2.0) return 0.5;
    return 0.1;
}

// 多径传播因子（简化）(Multipath propagation factor, simplified)
// 返回对信号功率的乘性因子 (Returns multiplicative factor for signal power)
struct multipath_model {
    double surface_reflection_coeff = -0.8;  // 地面反射系数，海面典型值 (Surface reflection coefficient, typical for sea)
    double surface_roughness_m      = 0.0;   // 均方根粗糙度（m）(RMS surface roughness)

    // 计算多径传播因子 (Compute multipath propagation factor)
    double propagation_factor(double range_m, double h_tx_m, double h_rx_m,
                              double frequency_hz) const {
        double lambda = constants::speed_of_light / frequency_hz;

        // 直达路径相位 (Direct path phase)
        double R_direct = std::sqrt(range_m*range_m + std::pow(h_tx_m - h_rx_m, 2));

        // 反射路径（镜像法）(Reflected path via image method)
        double R_reflected = std::sqrt(range_m*range_m + std::pow(h_tx_m + h_rx_m, 2));

        double delta_R = R_reflected - R_direct;
        double phase_diff = 2.0 * constants::pi * delta_R / lambda;

        // 粗糙度削弱因子（Ament）(Roughness attenuation factor, Ament)
        double roughness_factor = 1.0;
        if (surface_roughness_m > 0.0) {
            double grazing = std::atan2(h_tx_m + h_rx_m, range_m);
            double g = 4.0 * constants::pi * surface_roughness_m * std::sin(grazing) / lambda;
            roughness_factor = std::exp(-0.5 * g * g);
        }

        double rho = std::abs(surface_reflection_coeff) * roughness_factor;

        // 直达与反射路径的相干叠加
        // F^2 = |1 + rho * exp(j*phase)|^2
        double f2 = 1.0 + rho*rho + 2.0*rho*std::cos(phase_diff + constants::pi);

        return f2;
    }
};

// 综合传播损耗参数 (Comprehensive propagation loss parameters)
struct propagation_params {
    double range_m;                        // 距离（m）(Range)
    double frequency_hz;                   // 频率（Hz）(Frequency)
    double elevation_rad      = 0.0;       // 俯仰角（rad）(Elevation angle)
    double rain_rate_mm_per_hr = 0.0;      // 降雨率（mm/h）(Rain rate)
    bool   include_atmospheric = true;     // 是否包含大气衰减 (Include atmospheric attenuation)
    bool   include_rain        = true;     // 是否包含雨衰减 (Include rain attenuation)
    bool   include_lens        = true;     // 是否包含透镜损耗 (Include lens loss)
};

// 计算综合传播损耗（dB）(Compute total propagation loss in dB)
inline double total_propagation_loss_db(const propagation_params& p) {
    double loss = free_space_loss_db(p.range_m, p.frequency_hz);

    if (p.include_atmospheric)
        loss += blake_attenuation::total_loss_db(p.range_m, p.frequency_hz, p.elevation_rad);

    if (p.include_rain && p.rain_rate_mm_per_hr > 0.0) {
        rain_attenuation rain;
        rain.rain_rate_mm_per_hr = p.rain_rate_mm_per_hr;
        loss += rain.loss_db(p.range_m, p.frequency_hz);
    }

    if (p.include_lens)
        loss += lens_loss_db(p.elevation_rad);

    return loss;
}

} // namespace xsf_math
