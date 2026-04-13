#pragma once

#include "../core/constants.hpp"
#include "propagation.hpp"
#include "antenna.hpp"
#include "marcum_swerling.hpp"
#include <cmath>

namespace xsf_math {

// 发射机参数
struct transmitter_params {
    double peak_power_w       = 1000.0;   // 峰值发射功率（W）
    double frequency_hz       = 10.0e9;   // 工作频率
    double pulse_width_s      = 1.0e-6;   // 脉宽
    double prf_hz             = 1000.0;    // 脉冲重复频率
    double duty_cycle         = 0.001;     // pulse_width * PRF
    double bandwidth_hz       = 1.0e6;     // 接收机带宽
    double system_loss_db     = 3.0;       // 系统总损耗
};

// 接收机参数
struct receiver_params {
    double noise_figure_db   = 3.0;       // 噪声系数
    double noise_temp_k      = 290.0;     // 系统噪声温度
    double bandwidth_hz      = 1.0e6;     // 接收机带宽
    double processing_gain_db = 0.0;      // 信号处理增益

    // 热噪声功率（W）
    double noise_power_w() const {
        double nf = db_to_linear(noise_figure_db);
        return constants::boltzmann_k * noise_temp_k * bandwidth_hz * nf;
    }
};

// 雷达几何参数
struct radar_geometry {
    double range_m;                        // 到目标的斜距
    double target_rcs_m2;                  // 目标 RCS（m^2）
    double tx_antenna_gain_db = 30.0;      // 指向目标的发射天线增益
    double rx_antenna_gain_db = 30.0;      // 指向目标的接收天线增益
    double elevation_rad      = 0.0;       // 指向目标的俯仰角
};

// 单基地雷达方程结果
struct radar_equation_result {
    double signal_power_w;     // 接收信号功率
    double noise_power_w;      // 噪声功率
    double snr_linear;         // 信噪比（线性值）
    double snr_db;             // 信噪比（dB）
    double max_range_m;        // 给定参数下的最大探测距离
};

// 单基地雷达方程
// Pr = (Pt * Gt * Gr * lambda^2 * sigma) / ((4*pi)^3 * R^4 * L)
inline radar_equation_result monostatic_radar_equation(
        const transmitter_params& tx,
        const receiver_params& rx,
        const radar_geometry& geom) {

    double lambda = constants::speed_of_light / tx.frequency_hz;
    double four_pi_cubed = std::pow(4.0 * constants::pi, 3);
    double L = db_to_linear(tx.system_loss_db);
    double Gt = db_to_linear(geom.tx_antenna_gain_db);
    double Gr = db_to_linear(geom.rx_antenna_gain_db);
    double R4 = std::pow(geom.range_m, 4);

    double numerator = tx.peak_power_w * Gt * Gr * lambda * lambda * geom.target_rcs_m2;
    double denominator = four_pi_cubed * R4 * L;

    radar_equation_result r;
    r.signal_power_w = (denominator > 0.0) ? numerator / denominator : 0.0;
    r.noise_power_w  = rx.noise_power_w();

    // 应用处理增益
    double proc_gain = db_to_linear(rx.processing_gain_db);
    r.signal_power_w *= proc_gain;

    r.snr_linear = (r.noise_power_w > 0.0) ? r.signal_power_w / r.noise_power_w : 0.0;
    r.snr_db     = (r.snr_linear > 0.0) ? linear_to_db(r.snr_linear) : -999.0;

    // 最大作用距离（SNR = 1，0 dB 门限）
    double max_R4 = numerator / (four_pi_cubed * L * r.noise_power_w);
    r.max_range_m = (max_R4 > 0.0) ? std::pow(max_R4, 0.25) : 0.0;

    return r;
}

// 双基地雷达方程
// Pr = (Pt * Gt * Gr * lambda^2 * sigma) / ((4*pi)^3 * Rt^2 * Rr^2 * L)
struct bistatic_geometry {
    double range_tx_m;           // 发射机到目标的距离
    double range_rx_m;           // 目标到接收机的距离
    double target_rcs_m2;
    double tx_antenna_gain_db;
    double rx_antenna_gain_db;
    double elevation_rad = 0.0;
};

inline radar_equation_result bistatic_radar_equation(
        const transmitter_params& tx,
        const receiver_params& rx,
        const bistatic_geometry& geom) {

    double lambda = constants::speed_of_light / tx.frequency_hz;
    double four_pi_cubed = std::pow(4.0 * constants::pi, 3);
    double L = db_to_linear(tx.system_loss_db);
    double Gt = db_to_linear(geom.tx_antenna_gain_db);
    double Gr = db_to_linear(geom.rx_antenna_gain_db);
    double R2t_R2r = geom.range_tx_m * geom.range_tx_m * geom.range_rx_m * geom.range_rx_m;

    double numerator = tx.peak_power_w * Gt * Gr * lambda * lambda * geom.target_rcs_m2;
    double denominator = four_pi_cubed * R2t_R2r * L;

    radar_equation_result r;
    r.signal_power_w = (denominator > 0.0) ? numerator / denominator : 0.0;
    r.noise_power_w  = rx.noise_power_w();
    r.snr_linear = (r.noise_power_w > 0.0) ? r.signal_power_w / r.noise_power_w : 0.0;
    r.snr_db     = (r.snr_linear > 0.0) ? linear_to_db(r.snr_linear) : -999.0;
    r.max_range_m = 0.0;  // 双基地情形下不直接适用

    return r;
}

// 含杂波和干扰时的信噪比：SNR = S / (C + J + N)
struct snr_with_interference {
    double signal_power_w  = 0.0;
    double clutter_power_w = 0.0;
    double jamming_power_w = 0.0;
    double noise_power_w   = 0.0;

    double snr_linear() const {
        double total_noise = clutter_power_w + jamming_power_w + noise_power_w;
        return (total_noise > 0.0) ? signal_power_w / total_noise : 0.0;
    }

    double snr_db() const {
        double s = snr_linear();
        return (s > 0.0) ? linear_to_db(s) : -999.0;
    }

    // 信杂比
    double scr_db() const {
        return (clutter_power_w > 0.0) ? linear_to_db(signal_power_w / clutter_power_w) : 999.0;
    }

    // 信干比
    double sjr_db() const {
        return (jamming_power_w > 0.0) ? linear_to_db(signal_power_w / jamming_power_w) : 999.0;
    }
};

// 基于完整雷达链路计算探测概率
inline double compute_detection_probability(
        const transmitter_params& tx,
        const receiver_params& rx,
        const radar_geometry& geom,
        const marcum_swerling& detector) {
    auto result = monostatic_radar_equation(tx, rx, geom);
    return detector.compute_pd(result.snr_linear);
}

// 给定 Pd 下的作用距离
inline double compute_detection_range(
        const transmitter_params& tx,
        const receiver_params& rx,
        double target_rcs_m2,
        double antenna_gain_db,
        const marcum_swerling& detector,
        double required_pd = 0.5) {

    double required_snr = detector.required_snr(required_pd);
    if (required_snr <= 0.0) return 0.0;

    double lambda = constants::speed_of_light / tx.frequency_hz;
    double four_pi_cubed = std::pow(4.0 * constants::pi, 3);
    double L = db_to_linear(tx.system_loss_db);
    double G = db_to_linear(antenna_gain_db);
    double N = rx.noise_power_w();

    double numerator = tx.peak_power_w * G * G * lambda * lambda * target_rcs_m2;
    double denominator = four_pi_cubed * L * N * required_snr;

    return (denominator > 0.0) ? std::pow(numerator / denominator, 0.25) : 0.0;
}

} // namespace xsf_math
