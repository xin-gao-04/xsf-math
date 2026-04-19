#pragma once

#include "../core/constants.hpp"
#include "propagation.hpp"
#include "antenna.hpp"
#include "marcum_swerling.hpp"
#include <xsf_common/log.hpp>
#include <cmath>

namespace xsf_math {

// 发射机参数 (Transmitter parameters)
struct transmitter_params {
    double peak_power_w       = 1000.0;   // 峰值发射功率（W）(Peak transmit power)
    double frequency_hz       = 10.0e9;   // 工作频率（Hz）(Operating frequency)
    double pulse_width_s      = 1.0e-6;   // 脉宽（s）(Pulse width)
    double prf_hz             = 1000.0;   // 脉冲重复频率（Hz）(Pulse repetition frequency)
    double duty_cycle         = 0.001;    // 占空比：pulse_width * PRF (Duty cycle)
    double bandwidth_hz       = 1.0e6;    // 接收机带宽（Hz）(Receiver bandwidth)
    double system_loss_db     = 3.0;      // 系统总损耗（dB）(Total system loss)
};

// 接收机参数 (Receiver parameters)
struct receiver_params {
    double noise_figure_db   = 3.0;       // 噪声系数（dB）(Noise figure)
    double noise_temp_k      = 290.0;     // 系统噪声温度（K）(System noise temperature)
    double bandwidth_hz      = 1.0e6;     // 接收机带宽（Hz）(Receiver bandwidth)
    double processing_gain_db = 0.0;      // 信号处理增益（dB）(Processing gain)

    // 热噪声功率（W）(Thermal noise power in Watts)
    double noise_power_w() const {
        double nf = db_to_linear(noise_figure_db);
        return constants::boltzmann_k * noise_temp_k * bandwidth_hz * nf;
    }
};

// 雷达几何参数 (Radar geometry parameters)
struct radar_geometry {
    double range_m;                        // 到目标的斜距（m）(Slant range to target)
    double target_rcs_m2;                  // 目标雷达截面积（m^2）(Target RCS)
    double tx_antenna_gain_db = 30.0;      // 指向目标的发射天线增益（dBi）(Tx antenna gain toward target)
    double rx_antenna_gain_db = 30.0;      // 指向目标的接收天线增益（dBi）(Rx antenna gain toward target)
    double elevation_rad      = 0.0;       // 指向目标的俯仰角（rad）(Elevation angle to target)
};

// 单基地雷达方程结果 (Monostatic radar equation result)
struct radar_equation_result {
    double signal_power_w;     // 接收信号功率（W）(Received signal power)
    double noise_power_w;      // 噪声功率（W）(Noise power)
    double snr_linear;         // 信噪比（线性值）(SNR linear)
    double snr_db;             // 信噪比（dB）(SNR in dB)
    double max_range_m;        // 给定参数下的最大探测距离（m）(Maximum detection range)
};

// 单基地雷达方程 (Monostatic radar equation)
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

    // 应用处理增益 (Apply processing gain)
    double proc_gain = db_to_linear(rx.processing_gain_db);
    r.signal_power_w *= proc_gain;

    r.snr_linear = (r.noise_power_w > 0.0) ? r.signal_power_w / r.noise_power_w : 0.0;
    r.snr_db     = (r.snr_linear > 0.0) ? linear_to_db(r.snr_linear) : -999.0;

    // 最大作用距离（SNR = 1，0 dB 门限）(Max range at 0 dB SNR threshold)
    double max_R4 = numerator / (four_pi_cubed * L * r.noise_power_w);
    r.max_range_m = (max_R4 > 0.0) ? std::pow(max_R4, 0.25) : 0.0;

    XSF_LOG_TRACE("monostatic radar: R={:.0f}m sigma={:.3f}m^2 Pr={:.3e}W Pn={:.3e}W SNR={:.2f}dB",
                  geom.range_m,
                  geom.target_rcs_m2,
                  r.signal_power_w,
                  r.noise_power_w,
                  r.snr_db);

    return r;
}

// 双基地雷达方程 (Bistatic radar equation)
// Pr = (Pt * Gt * Gr * lambda^2 * sigma) / ((4*pi)^3 * Rt^2 * Rr^2 * L)
struct bistatic_geometry {
    double range_tx_m;           // 发射机到目标的距离（m）(Range from transmitter to target)
    double range_rx_m;           // 目标到接收机的距离（m）(Range from target to receiver)
    double target_rcs_m2;        // 目标雷达截面积（m^2）(Target RCS)
    double tx_antenna_gain_db;   // 发射天线增益（dBi）(Tx antenna gain)
    double rx_antenna_gain_db;   // 接收天线增益（dBi）(Rx antenna gain)
    double elevation_rad = 0.0;  // 俯仰角（rad）(Elevation angle)
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
    r.max_range_m = 0.0;  // 双基地情形下不直接适用 (Not directly applicable for bistatic)

    XSF_LOG_TRACE("bistatic radar: Rt={:.0f}m Rr={:.0f}m sigma={:.3f}m^2 Pr={:.3e}W SNR={:.2f}dB",
                  geom.range_tx_m,
                  geom.range_rx_m,
                  geom.target_rcs_m2,
                  r.signal_power_w,
                  r.snr_db);

    return r;
}

// 含杂波和干扰时的信噪比：SNR = S / (C + J + N) (SNR with clutter and jamming)
struct snr_with_interference {
    double signal_power_w  = 0.0;  // 信号功率（W）(Signal power)
    double clutter_power_w = 0.0;  // 杂波功率（W）(Clutter power)
    double jamming_power_w = 0.0;  // 干扰功率（W）(Jamming power)
    double noise_power_w   = 0.0;  // 噪声功率（W）(Noise power)

    // 总信噪比（线性值）(Total SNR linear)
    double snr_linear() const {
        double total_noise = clutter_power_w + jamming_power_w + noise_power_w;
        return (total_noise > 0.0) ? signal_power_w / total_noise : 0.0;
    }

    // 总信噪比（dB）(Total SNR in dB)
    double snr_db() const {
        double s = snr_linear();
        return (s > 0.0) ? linear_to_db(s) : -999.0;
    }

    // 信杂比（dB）(Signal-to-clutter ratio in dB)
    double scr_db() const {
        return (clutter_power_w > 0.0) ? linear_to_db(signal_power_w / clutter_power_w) : 999.0;
    }

    // 信干比（dB）(Signal-to-jamming ratio in dB)
    double sjr_db() const {
        return (jamming_power_w > 0.0) ? linear_to_db(signal_power_w / jamming_power_w) : 999.0;
    }
};

// 基于完整雷达链路计算探测概率 (Compute detection probability from complete radar link)
inline double compute_detection_probability(
        const transmitter_params& tx,
        const receiver_params& rx,
        const radar_geometry& geom,
        const marcum_swerling& detector) {
    auto result = monostatic_radar_equation(tx, rx, geom);
    const double pd = detector.compute_pd(result.snr_linear);
    XSF_LOG_DEBUG("detection probability: R={:.0f}m SNR={:.2f}dB Pd={:.4f}",
                  geom.range_m,
                  result.snr_db,
                  pd);
    return pd;
}

// 给定 Pd 下的作用距离（m）(Compute detection range for required Pd)
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
    const double detection_range_m =
        (denominator > 0.0) ? std::pow(numerator / denominator, 0.25) : 0.0;
    XSF_LOG_DEBUG("detection range: sigma={:.3f}m^2 required Pd={:.3f} required SNR={:.2f}dB range={:.0f}m",
                  target_rcs_m2,
                  required_pd,
                  linear_to_db(required_snr),
                  detection_range_m);
    return detection_range_m;
}

} // namespace xsf_math
