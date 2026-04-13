#pragma once

#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Surface clutter models

// Ground clutter RCS per unit area (sigma_0) models
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

// Constant-gamma clutter model
// sigma_0 = gamma * sin(grazing_angle)
struct constant_gamma_clutter {
    double gamma_db = -20.0;  // clutter coefficient (dB)

    double sigma0(double grazing_angle_rad) const {
        double gamma = db_to_linear(gamma_db);
        return gamma * std::sin(std::abs(grazing_angle_rad));
    }

    // Get typical gamma for surface type
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

// Clutter area computation
struct clutter_geometry {
    double range_m;
    double grazing_angle_rad;
    double azimuth_beamwidth_rad;
    double pulse_width_s;
    double range_resolution_m;  // c * tau / 2

    // Clutter patch area for pulse-limited radar
    double patch_area_pulse() const {
        return range_m * azimuth_beamwidth_rad * range_resolution_m;
    }

    // Clutter patch area for beam-limited radar
    double patch_area_beam(double elevation_beamwidth_rad) const {
        return range_m * range_m * azimuth_beamwidth_rad * elevation_beamwidth_rad;
    }
};

// Clutter power received by radar
inline double clutter_power_w(
        double tx_power_w,
        double tx_gain_linear,
        double rx_gain_linear,
        double frequency_hz,
        double range_m,
        double sigma0,           // clutter RCS per unit area
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

// MTI improvement factor (simplified)
// Reduction of clutter power after Moving Target Indication processing
inline double mti_improvement_factor_db(int num_pulses, double clutter_velocity_spread_mps,
                                         double prf_hz, double frequency_hz) {
    // MTI improvement is approximately:
    // I = N^2 * (PRF / (2 * sigma_v / lambda))^2 for single canceller
    double lambda = constants::speed_of_light / frequency_hz;
    if (clutter_velocity_spread_mps < 1e-10) return 60.0;  // nearly stationary clutter

    double sigma_f = clutter_velocity_spread_mps / lambda;  // spectral spread in Hz
    double ratio = prf_hz / (2.0 * sigma_f);

    double improvement = static_cast<double>(num_pulses * num_pulses) * ratio * ratio;
    return std::min(linear_to_db(improvement), 60.0);  // cap at 60 dB
}

// Pulse-Doppler clutter rejection
inline double pd_clutter_rejection_db(double mti_db, double filter_rejection_db = 30.0) {
    return mti_db + filter_rejection_db;
}

} // namespace xsf_math
