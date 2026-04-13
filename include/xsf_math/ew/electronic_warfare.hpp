#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Electronic Warfare (EW) computation models

// Jammer parameters
struct jammer_params {
    double erp_w           = 1000.0;   // effective radiated power (W)
    double bandwidth_hz    = 1.0e9;    // jamming bandwidth
    double antenna_gain_db = 10.0;     // jammer antenna gain toward radar

    // Jammer power density at radar receiver
    double power_density(double range_m) const {
        if (range_m < 1e-10) return 0.0;
        return erp_w / (4.0 * constants::pi * range_m * range_m);
    }
};

// Self-screening jamming (SSJ)
// Target carries its own jammer; radar sees jamming from target direction
// J/S = (Pj * Gj * 4 * pi * R^2) / (Pt * Gt * Gr * lambda^2 * sigma / ((4pi)^3 * R^4))
//     = (Pj * Gj * 4pi * R^2 * (4pi)^3 * R^4) / (Pt * Gt * Gr * lambda^2 * sigma * ...)
// Simplified: J/S = (Pj*Gj * 4*pi * R^2) / (Pt*Gt*sigma / (4*pi*R^2)) * (Bj/Br)
struct self_screening_jam {
    // Jam-to-Signal ratio (linear) for self-screening case
    // J/S ∝ R^2 (jammer advantage increases with range)
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

        // Signal power at receiver (numerator of radar equation * R^4 / R^4)
        double S_num = radar_tx_power_w * radar_tx_gain_linear * radar_rx_gain_linear *
                       lambda * lambda * target_rcs_m2;
        double S_den = std::pow(four_pi, 3) * std::pow(range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // Jamming power at receiver
        double J_num = jammer_erp_w * radar_rx_gain_linear * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * range_m * range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        // Bandwidth ratio correction
        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }

    // Burnthrough range: range at which J/S = 1 (radar overcomes jamming)
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

        // At burnthrough: J/S = 1
        // R_bt^2 = (Pj*Gj_towards_radar * 4*pi) / (Pt*Gt*sigma/(4*pi*lambda^2)) * (Br/Bj)
        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;

        double R2 = (jammer_erp_w * four_pi * bw_ratio) /
                    (radar_tx_power_w * radar_tx_gain_linear * target_rcs_m2 /
                     (four_pi * lambda * lambda));

        return (R2 > 0.0) ? std::sqrt(R2) : 0.0;
    }
};

// Stand-off jamming (SOJ)
// Jammer is at different location from target
struct stand_off_jam {
    // J/S for stand-off case
    // J/S = (Pj*Gj*Rt^4) / (Pt*Gt*sigma*Rj^2/(4*pi*lambda^2)) * (Br/Bj)
    static double jam_to_signal(
            double jammer_erp_w,
            double jammer_bw_hz,
            double jammer_range_m,      // jammer to radar range
            double radar_tx_power_w,
            double radar_tx_gain_linear,
            double radar_rx_gain_toward_jammer,  // radar gain in jammer direction
            double radar_freq_hz,
            double radar_bw_hz,
            double target_rcs_m2,
            double target_range_m) {

        double lambda = constants::speed_of_light / radar_freq_hz;
        double four_pi = 4.0 * constants::pi;

        // Signal from target
        double S_num = radar_tx_power_w * radar_tx_gain_linear * target_rcs_m2 *
                       lambda * lambda;
        double S_den = std::pow(four_pi, 3) * std::pow(target_range_m, 4);
        double S = (S_den > 0) ? S_num / S_den : 0.0;

        // Jamming power at radar
        double J_num = jammer_erp_w * radar_rx_gain_toward_jammer * lambda * lambda;
        double J_den = std::pow(four_pi, 2) * jammer_range_m * jammer_range_m;
        double J = (J_den > 0) ? J_num / J_den : 0.0;

        double bw_ratio = (jammer_bw_hz > radar_bw_hz) ? radar_bw_hz / jammer_bw_hz : 1.0;
        J *= bw_ratio;

        return (S > 0.0) ? J / S : 1e10;
    }
};

// SNR degradation factors from EW effects
// Applied multiplicatively to the baseline SNR
struct ew_degradation {
    double jamming_power_gain  = 1.0;  // multiplicative factor on jamming power
    double noise_multiplier    = 1.0;  // multiplicative factor on noise floor
    double blanking_factor     = 1.0;  // fraction of time radar is not blanked (0-1)

    // Apply to baseline SNR
    double degrade_snr(double baseline_snr_linear,
                        double js_ratio_linear = 0.0) const {
        // SNR = S / (N * noise_mult + J * jam_gain)
        // where J = js_ratio * S
        double effective_noise = noise_multiplier + js_ratio_linear * jamming_power_gain;
        double degraded = baseline_snr_linear / effective_noise;
        return degraded * blanking_factor;
    }
};

// Chaff RCS model (simplified)
inline double chaff_rcs_m2(double num_dipoles, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    // Each dipole has RCS ≈ 0.86 * lambda^2 / (4*pi) at resonance
    double rcs_per_dipole = 0.86 * lambda * lambda / (4.0 * constants::pi);
    return num_dipoles * rcs_per_dipole;
}

// Decoy effectiveness: probability radar tracks decoy instead of target
inline double decoy_effectiveness(double decoy_rcs_m2, double target_rcs_m2,
                                   double angular_separation_rad,
                                   double radar_beamwidth_rad) {
    // If decoy is within radar beam with the target
    if (angular_separation_rad > radar_beamwidth_rad) return 0.0;

    // Simple model: probability proportional to RCS ratio
    double total_rcs = decoy_rcs_m2 + target_rcs_m2;
    if (total_rcs <= 0.0) return 0.0;
    return decoy_rcs_m2 / total_rcs;
}

} // namespace xsf_math
