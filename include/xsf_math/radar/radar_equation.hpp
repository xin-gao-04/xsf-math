#pragma once

#include "../core/constants.hpp"
#include "propagation.hpp"
#include "antenna.hpp"
#include "marcum_swerling.hpp"
#include <cmath>

namespace xsf_math {

// Transmitter parameters
struct transmitter_params {
    double peak_power_w       = 1000.0;   // peak transmit power (W)
    double frequency_hz       = 10.0e9;   // operating frequency
    double pulse_width_s      = 1.0e-6;   // pulse width
    double prf_hz             = 1000.0;    // pulse repetition frequency
    double duty_cycle         = 0.001;     // pulse_width * PRF
    double bandwidth_hz       = 1.0e6;     // receiver bandwidth
    double system_loss_db     = 3.0;       // total system losses
};

// Receiver parameters
struct receiver_params {
    double noise_figure_db   = 3.0;       // noise figure
    double noise_temp_k      = 290.0;     // system noise temperature
    double bandwidth_hz      = 1.0e6;     // receiver bandwidth
    double processing_gain_db = 0.0;      // signal processing gain

    // Thermal noise power (W)
    double noise_power_w() const {
        double nf = db_to_linear(noise_figure_db);
        return constants::boltzmann_k * noise_temp_k * bandwidth_hz * nf;
    }
};

// Radar geometry
struct radar_geometry {
    double range_m;                        // slant range to target
    double target_rcs_m2;                  // target RCS in m^2
    double tx_antenna_gain_db = 30.0;      // transmit antenna gain toward target
    double rx_antenna_gain_db = 30.0;      // receive antenna gain toward target
    double elevation_rad      = 0.0;       // elevation angle to target
};

// Monostatic radar equation result
struct radar_equation_result {
    double signal_power_w;     // received signal power
    double noise_power_w;      // noise power
    double snr_linear;         // signal-to-noise ratio (linear)
    double snr_db;             // SNR in dB
    double max_range_m;        // maximum detection range for given parameters
};

// Monostatic radar equation
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

    // Apply processing gain
    double proc_gain = db_to_linear(rx.processing_gain_db);
    r.signal_power_w *= proc_gain;

    r.snr_linear = (r.noise_power_w > 0.0) ? r.signal_power_w / r.noise_power_w : 0.0;
    r.snr_db     = (r.snr_linear > 0.0) ? linear_to_db(r.snr_linear) : -999.0;

    // Maximum range (SNR = 1, 0 dB threshold)
    double max_R4 = numerator / (four_pi_cubed * L * r.noise_power_w);
    r.max_range_m = (max_R4 > 0.0) ? std::pow(max_R4, 0.25) : 0.0;

    return r;
}

// Bistatic radar equation
// Pr = (Pt * Gt * Gr * lambda^2 * sigma) / ((4*pi)^3 * Rt^2 * Rr^2 * L)
struct bistatic_geometry {
    double range_tx_m;           // range from TX to target
    double range_rx_m;           // range from target to RX
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
    r.max_range_m = 0.0;  // not directly applicable for bistatic

    return r;
}

// SNR with clutter and jamming: SNR = S / (C + J + N)
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

    // Signal-to-clutter ratio
    double scr_db() const {
        return (clutter_power_w > 0.0) ? linear_to_db(signal_power_w / clutter_power_w) : 999.0;
    }

    // Signal-to-jamming ratio
    double sjr_db() const {
        return (jamming_power_w > 0.0) ? linear_to_db(signal_power_w / jamming_power_w) : 999.0;
    }
};

// Detection probability from full radar chain
inline double compute_detection_probability(
        const transmitter_params& tx,
        const receiver_params& rx,
        const radar_geometry& geom,
        const marcum_swerling& detector) {
    auto result = monostatic_radar_equation(tx, rx, geom);
    return detector.compute_pd(result.snr_linear);
}

// Range for given Pd
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
