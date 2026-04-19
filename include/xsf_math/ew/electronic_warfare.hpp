#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 电子战（Electronic Warfare, EW）计算模型 (Electronic Warfare computation models)

// 干扰机参数 (Jammer parameters)
struct jammer_params {
    double erp_w           = 1000.0;   // 有效辐射功率（Effective Radiated Power, ERP），单位 W
    double bandwidth_hz    = 1.0e9;    // 干扰带宽（Jamming bandwidth），单位 Hz
    double antenna_gain_db = 10.0;     // 朝向雷达的干扰天线增益（Antenna gain toward radar），单位 dB

    // 雷达接收端的干扰机功率密度（Power density at radar receiver）
    // 单位：W/m^2
    double power_density(double range_m) const {
        if (range_m < 1e-10) return 0.0;
        return erp_w / (4.0 * constants::pi * range_m * range_m);
    }
};

// 自卫式干扰（Self-Screening Jamming, SSJ）
// 目标携带自己的干扰机；雷达从目标方向看到干扰 (Target carries its own jammer; radar sees jamming from target direction)
// J/S = (Pj * Gj * 4 * pi * R^2) / (Pt * Gt * Gr * lambda^2 * sigma / ((4pi)^3 * R^4))
//     = (Pj * Gj * 4pi * R^2 * (4pi)^3 * R^4) / (Pt * Gt * Gr * lambda^2 * sigma * ...)
// 简化后：J/S = (Pj*Gj * 4*pi * R^2) / (Pt*Gt*sigma / (4*pi*R^2)) * (Bj/Br)
struct self_screening_jam {
    // 自卫式情形下的干信比（Jam-to-Signal ratio, J/S），线性值（linear scale）
    // J/S ∝ R^2（干扰优势随距离增加）
    static double jam_to_signal(
            double jammer_erp_w,              // 干扰机有效辐射功率（W）
            double jammer_bw_hz,              // 干扰带宽（Hz）
            double radar_tx_power_w,          // 雷达发射功率（W）
            double radar_tx_gain_linear,      // 雷达发射增益（线性值）
            double radar_rx_gain_linear,      // 雷达接收增益（线性值）
            double radar_freq_hz,             // 雷达频率（Hz）
            double radar_bw_hz,               // 雷达带宽（Hz）
            double target_rcs_m2,             // 目标雷达截面积（Radar Cross Section, RCS，m^2）
            double range_m) {                 // 雷达到目标距离（m）

        double lambda = constants::speed_of_light / radar_freq_hz;   // 波长（wavelength，m）
        double four_pi = 4.0 * constants::pi;

        // 接收端信号功率（雷达方程分子 * R^4 / R^4）
        // Signal power at receiver (radar equation numerator)
        double S_num = radar_tx_power_w * radar_tx_gain_linear * radar_rx_gain_linear *
                       lambda * lambda * target_rcs_m2;
        double S_den = std::pow(four_pi, 3) * std::pow(range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // 接收端干扰功率（Jamming power at receiver）
        double J_num = jammer_erp_w * radar_rx_gain_linear * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * range_m * range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        // 带宽比修正（Bandwidth ratio correction）
        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }

    // 穿透距离（Burn-through range）：J/S = 1 时的距离（雷达压制干扰条件下）
    static double burnthrough_range_m(
            double jammer_erp_w,          // 干扰机有效辐射功率（W）
            double jammer_bw_hz,          // 干扰带宽（Hz）
            double radar_tx_power_w,      // 雷达发射功率（W）
            double radar_tx_gain_linear,  // 雷达发射增益（线性值）
            double radar_freq_hz,         // 雷达频率（Hz）
            double radar_bw_hz,           // 雷达带宽（Hz）
            double target_rcs_m2) {       // 目标雷达截面积（m^2）

        double lambda = constants::speed_of_light / radar_freq_hz;   // 波长（m）
        double four_pi = 4.0 * constants::pi;

        // 在穿透点：J/S = 1
        // R_bt^2 = (Pj*Gj_towards_radar * 4*pi) / (Pt*Gt*sigma/(4*pi*lambda^2)) * (Br/Bj)
        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;

        double R2 = (jammer_erp_w * four_pi * bw_ratio) /
                    (radar_tx_power_w * radar_tx_gain_linear * target_rcs_m2 /
                     (four_pi * lambda * lambda));

        return (R2 > 0.0) ? std::sqrt(R2) : 0.0;
    }
};

// 站外干扰（Stand-Off Jamming, SOJ）
// 干扰机与目标位置不同 (Jammer located at a different position from the target)
struct stand_off_jam {
    // 站外情形的 J/S（Jam-to-Signal ratio for stand-off jamming）
    // J/S = (Pj*Gj*Rt^4) / (Pt*Gt*sigma*Rj^2/(4*pi*lambda^2)) * (Br/Bj)
    static double jam_to_signal(
            double jammer_erp_w,                  // 干扰机有效辐射功率（W）
            double jammer_bw_hz,                  // 干扰带宽（Hz）
            double jammer_range_m,                // 干扰机到雷达的距离（m）
            double radar_tx_power_w,              // 雷达发射功率（W）
            double radar_tx_gain_linear,          // 雷达发射增益（线性值）
            double radar_rx_gain_toward_jammer,   // 雷达在干扰机方向的接收增益（线性值）
            double radar_freq_hz,                 // 雷达频率（Hz）
            double radar_bw_hz,                   // 雷达带宽（Hz）
            double target_rcs_m2,                 // 目标雷达截面积（m^2）
            double target_range_m) {              // 雷达到目标距离（m）

        double lambda = constants::speed_of_light / radar_freq_hz;   // 波长（m）
        double four_pi = 4.0 * constants::pi;

        // 来自目标的信号（Signal from target）
        double S_num = radar_tx_power_w * radar_tx_gain_linear * target_rcs_m2 *
                       lambda * lambda;
        double S_den = std::pow(four_pi, 3) * std::pow(target_range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // 雷达处的干扰功率（Jamming power at radar）
        double J_num = jammer_erp_w * radar_rx_gain_toward_jammer * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * jammer_range_m * jammer_range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }
};

// 由电子战效应引起的 SNR 降级因子（SNR degradation factor caused by EW effects）
// 以乘法方式作用于基线 SNR (Multiplicative factor applied to baseline SNR)
struct ew_degradation {
    double jamming_power_gain  = 1.0;  // 干扰功率乘子（Jamming power multiplier）
    double noise_multiplier    = 1.0;  // 噪声底乘子（Noise floor multiplier）
    double blanking_factor     = 1.0;  // 雷达未被压制的时间比例（0-1）（Fraction of time radar is not blanked, 0-1）

    // 作用于基线 SNR（Apply to baseline SNR）
    double degrade_snr(double baseline_snr_linear,    // 基线信噪比（线性值）
                        double js_ratio_linear = 0.0) const {
        // SNR = S / (N * noise_mult + J * jam_gain)
        // 其中 J = js_ratio * S
        double effective_noise = noise_multiplier + js_ratio_linear * jamming_power_gain;
        double degraded = baseline_snr_linear / effective_noise;
        return degraded * blanking_factor;
    }
};

// 箔条 RCS 模型（简化）（Chaff RCS model, simplified）
// num_dipoles: 偶极子数量；frequency_hz: 频率（Hz）
inline double chaff_rcs_m2(double num_dipoles, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;   // 波长（m）
    // 每个偶极子的共振 RCS 约为 0.86 * lambda^2 / (4*pi)
    // Resonant RCS per dipole
    double rcs_per_dipole = 0.86 * lambda * lambda / (4.0 * constants::pi);
    return num_dipoles * rcs_per_dipole;
}

// 诱饵有效性：雷达跟踪诱饵而非目标的概率（Decoy effectiveness: probability radar tracks decoy instead of target）
inline double decoy_effectiveness(double decoy_rcs_m2,          // 诱饵雷达截面积（m^2）
                                   double target_rcs_m2,          // 目标雷达截面积（m^2）
                                   double angular_separation_rad, // 诱饵与目标的角间距（rad）
                                   double radar_beamwidth_rad) {  // 雷达波束宽度（rad）
    // 如果诱饵与目标一起处于雷达波束内
    if (angular_separation_rad > radar_beamwidth_rad) return 0.0;

    // 简化模型：概率与 RCS 比例成正比（Simplified model: probability proportional to RCS ratio）
    double total_rcs = decoy_rcs_m2 + target_rcs_m2;
    if (total_rcs <= 0.0) return 0.0;
    return decoy_rcs_m2 / total_rcs;
}

// 基于参考回波的缩放关系，计算假目标等效 RCS。（False target equivalent RCS based on reference echo scaling）
// sigma_eq = sigma_ref * (SNR_actual / SNR_ref)
inline double false_target_equivalent_rcs(double actual_snr_linear,       // 实际信噪比（线性值）
                                          double reference_snr_linear,    // 参考信噪比（线性值）
                                          double reference_rcs_m2 = 1.0) {// 参考雷达截面积（m^2）
    if (actual_snr_linear <= 0.0 || reference_snr_linear <= 0.0 || reference_rcs_m2 <= 0.0) {
        return 0.0;
    }
    return reference_rcs_m2 * (actual_snr_linear / reference_snr_linear);
}

// 假目标位置沿视线方向使用半量程偏置。（False target range offset along line-of-sight using half-range bias）
inline double false_target_range_offset(double jammer_to_radar_range_m,           // 干扰机到雷达距离（m）
                                        double reference_jammer_to_radar_range_m) {// 参考干扰机到雷达距离（m）
    return 0.5 * (jammer_to_radar_range_m - reference_jammer_to_radar_range_m);
}

} // namespace xsf_math
