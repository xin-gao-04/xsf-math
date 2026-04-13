#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 电子战（EW）计算模型

// 干扰机参数
struct jammer_params {
    double erp_w           = 1000.0;   // 有效辐射功率（W）
    double bandwidth_hz    = 1.0e9;    // 干扰带宽
    double antenna_gain_db = 10.0;     // 朝向雷达的干扰天线增益

    // 雷达接收端的干扰机功率密度
    double power_density(double range_m) const {
        if (range_m < 1e-10) return 0.0;
        return erp_w / (4.0 * constants::pi * range_m * range_m);
    }
};

// 自卫式干扰（SSJ）
// 目标携带自己的干扰机；雷达从目标方向看到干扰
// J/S = (Pj * Gj * 4 * pi * R^2) / (Pt * Gt * Gr * lambda^2 * sigma / ((4pi)^3 * R^4))
//     = (Pj * Gj * 4pi * R^2 * (4pi)^3 * R^4) / (Pt * Gt * Gr * lambda^2 * sigma * ...)
// 简化后：J/S = (Pj*Gj * 4*pi * R^2) / (Pt*Gt*sigma / (4*pi*R^2)) * (Bj/Br)
struct self_screening_jam {
    // 自卫式情形下的干信比（线性）
    // J/S ∝ R^2（干扰优势随距离增加）
    static double jam_to_signal(
            double jammer_erp_w,
            double jammer_bw_hz,
            double radar_tx_power_w,
            double radar_tx_gain_linear,
            double radar_rx_gain_linear,
            double radar_freq_hz,
            double radar_bw_hz,
            double target_rcs_m2,
            double range_m) {

        double lambda = constants::speed_of_light / radar_freq_hz;
        double four_pi = 4.0 * constants::pi;

        // 接收端信号功率（雷达方程分子 * R^4 / R^4）
        double S_num = radar_tx_power_w * radar_tx_gain_linear * radar_rx_gain_linear *
                       lambda * lambda * target_rcs_m2;
        double S_den = std::pow(four_pi, 3) * std::pow(range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // 接收端干扰功率
        double J_num = jammer_erp_w * radar_rx_gain_linear * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * range_m * range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        // 带宽比修正
        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }

    // 穿透距离：J/S = 1 时的距离（雷达压制干扰）
    static double burnthrough_range_m(
            double jammer_erp_w,
            double jammer_bw_hz,
            double radar_tx_power_w,
            double radar_tx_gain_linear,
            double radar_freq_hz,
            double radar_bw_hz,
            double target_rcs_m2) {

        double lambda = constants::speed_of_light / radar_freq_hz;
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

// 站外干扰（SOJ）
// 干扰机与目标位置不同
struct stand_off_jam {
    // 站外情形的 J/S
    // J/S = (Pj*Gj*Rt^4) / (Pt*Gt*sigma*Rj^2/(4*pi*lambda^2)) * (Br/Bj)
    static double jam_to_signal(
            double jammer_erp_w,
            double jammer_bw_hz,
            double jammer_range_m,      // 干扰机到雷达的距离
            double radar_tx_power_w,
            double radar_tx_gain_linear,
            double radar_rx_gain_toward_jammer,  // 雷达在干扰机方向的增益
            double radar_freq_hz,
            double radar_bw_hz,
            double target_rcs_m2,
            double target_range_m) {

        double lambda = constants::speed_of_light / radar_freq_hz;
        double four_pi = 4.0 * constants::pi;

        // 来自目标的信号
        double S_num = radar_tx_power_w * radar_tx_gain_linear * target_rcs_m2 *
                       lambda * lambda;
        double S_den = std::pow(four_pi, 3) * std::pow(target_range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // 雷达处的干扰功率
        double J_num = jammer_erp_w * radar_rx_gain_toward_jammer * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * jammer_range_m * jammer_range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }
};

// 由电子战效应引起的 SNR 降级因子
// 以乘法方式作用于基线 SNR
struct ew_degradation {
    double jamming_power_gain  = 1.0;  // 干扰功率乘子
    double noise_multiplier    = 1.0;  // 噪声底乘子
    double blanking_factor     = 1.0;  // 雷达未被压制的时间比例（0-1）

    // 作用于基线 SNR
    double degrade_snr(double baseline_snr_linear,
                        double js_ratio_linear = 0.0) const {
        // SNR = S / (N * noise_mult + J * jam_gain)
        // 其中 J = js_ratio * S
        double effective_noise = noise_multiplier + js_ratio_linear * jamming_power_gain;
        double degraded = baseline_snr_linear / effective_noise;
        return degraded * blanking_factor;
    }
};

// 箔条 RCS 模型（简化）
inline double chaff_rcs_m2(double num_dipoles, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    // 每个偶极子的共振 RCS 约为 0.86 * lambda^2 / (4*pi)
    double rcs_per_dipole = 0.86 * lambda * lambda / (4.0 * constants::pi);
    return num_dipoles * rcs_per_dipole;
}

// 诱饵有效性：雷达跟踪诱饵而非目标的概率
inline double decoy_effectiveness(double decoy_rcs_m2, double target_rcs_m2,
                                   double angular_separation_rad,
                                   double radar_beamwidth_rad) {
    // 如果诱饵与目标一起处于雷达波束内
    if (angular_separation_rad > radar_beamwidth_rad) return 0.0;

    // 简化模型：概率与 RCS 比例成正比
    double total_rcs = decoy_rcs_m2 + target_rcs_m2;
    if (total_rcs <= 0.0) return 0.0;
    return decoy_rcs_m2 / total_rcs;
}

// 基于参考回波的缩放关系，计算假目标等效 RCS。
// 该关系直接来自 xsf-core 的 false target 幅度匹配逻辑：
// sigma_eq = sigma_ref * (SNR_actual / SNR_ref)
inline double false_target_equivalent_rcs(double actual_snr_linear,
                                          double reference_snr_linear,
                                          double reference_rcs_m2 = 1.0) {
    if (actual_snr_linear <= 0.0 || reference_snr_linear <= 0.0 || reference_rcs_m2 <= 0.0) {
        return 0.0;
    }
    return reference_rcs_m2 * (actual_snr_linear / reference_snr_linear);
}

// xsf-core 中假目标位置沿视线方向使用半量程偏置。
inline double false_target_range_offset(double jammer_to_radar_range_m,
                                        double reference_jammer_to_radar_range_m) {
    return 0.5 * (jammer_to_radar_range_m - reference_jammer_to_radar_range_m);
}

} // namespace xsf_math
