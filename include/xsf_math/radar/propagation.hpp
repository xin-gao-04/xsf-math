#pragma once

#include "../core/constants.hpp"
#include "../core/atmosphere.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Electromagnetic propagation loss models

// Free-space path loss (FSPL) in dB
// L_fs = 20*log10(4*pi*R/lambda)
inline double free_space_loss_db(double range_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    if (range_m < 1e-10 || lambda < 1e-30) return 0.0;
    double ratio = 4.0 * constants::pi * range_m / lambda;
    return 20.0 * std::log10(ratio);
}

// Two-ray (flat earth) propagation factor
// F^4 = gain relative to free space due to ground reflection
inline double two_ray_factor(double range_m, double h_tx_m, double h_rx_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    double arg = 4.0 * constants::pi * h_tx_m * h_rx_m / (lambda * range_m);
    double sin_val = std::sin(arg);
    double factor = 2.0 * sin_val;  // voltage factor
    return factor * factor;  // power factor (F^2)
}

// Blake's atmospheric attenuation model (dB/km as function of frequency and elevation)
struct blake_attenuation {
    // Simplified model: attenuation rate increases with frequency
    // and decreases with elevation (less atmosphere traversed at higher angles)
    static double rate_db_per_km(double frequency_hz, double elevation_rad) {
        double f_ghz = frequency_hz / 1.0e9;

        // Base attenuation rate (clear air, sea level)
        double alpha;
        if (f_ghz < 1.0) {
            alpha = 0.004;
        } else if (f_ghz < 10.0) {
            alpha = 0.004 + 0.002 * (f_ghz - 1.0);
        } else if (f_ghz < 40.0) {
            // Water vapor and oxygen absorption peaks
            alpha = 0.022 + 0.01 * std::pow((f_ghz - 10.0) / 30.0, 2.0);
        } else {
            alpha = 0.05;
        }

        // Elevation correction (less atmosphere at higher angles)
        double el_factor = 1.0;
        if (std::abs(elevation_rad) > 0.01) {
            el_factor = 1.0 / std::sin(std::abs(elevation_rad));
            el_factor = std::min(el_factor, 50.0);  // cap at horizon
        }

        return alpha * el_factor;
    }

    // Total atmospheric attenuation for given range and frequency
    static double total_loss_db(double range_m, double frequency_hz, double elevation_rad = 0.0) {
        return rate_db_per_km(frequency_hz, elevation_rad) * (range_m / 1000.0);
    }
};

// Rain attenuation model (ITU-R P.838)
struct rain_attenuation {
    double rain_rate_mm_per_hr = 0.0;  // 0 = clear

    double loss_db(double range_m, double frequency_hz) const {
        if (rain_rate_mm_per_hr <= 0.0) return 0.0;

        double f_ghz = frequency_hz / 1.0e9;

        // Simplified ITU model: gamma = k * R^alpha (dB/km)
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

// Lens loss (beam spreading due to atmospheric refraction)
inline double lens_loss_db(double elevation_rad) {
    // Simplified model: significant only near horizon
    if (std::abs(elevation_rad) > 10.0 * constants::deg_to_rad)
        return 0.0;
    double el_deg = std::abs(elevation_rad) * constants::rad_to_deg;
    if (el_deg < 0.5) return 1.0;
    if (el_deg < 2.0) return 0.5;
    return 0.1;
}

// Multipath propagation factor (simplified)
// Returns multiplicative factor on signal power
struct multipath_model {
    double surface_reflection_coeff = -0.8;  // typical for sea surface
    double surface_roughness_m      = 0.0;   // RMS roughness

    double propagation_factor(double range_m, double h_tx_m, double h_rx_m,
                              double frequency_hz) const {
        double lambda = constants::speed_of_light / frequency_hz;

        // Direct path phase
        double R_direct = std::sqrt(range_m*range_m + std::pow(h_tx_m - h_rx_m, 2));

        // Reflected path (image method)
        double R_reflected = std::sqrt(range_m*range_m + std::pow(h_tx_m + h_rx_m, 2));

        double delta_R = R_reflected - R_direct;
        double phase_diff = 2.0 * constants::pi * delta_R / lambda;

        // Roughness reduction factor (Ament)
        double roughness_factor = 1.0;
        if (surface_roughness_m > 0.0) {
            double grazing = std::atan2(h_tx_m + h_rx_m, range_m);
            double g = 4.0 * constants::pi * surface_roughness_m * std::sin(grazing) / lambda;
            roughness_factor = std::exp(-0.5 * g * g);
        }

        double rho = std::abs(surface_reflection_coeff) * roughness_factor;

        // Coherent sum of direct + reflected
        // F^2 = |1 + rho * exp(j*phase)|^2
        double f2 = 1.0 + rho*rho + 2.0*rho*std::cos(phase_diff + constants::pi);

        return f2;
    }
};

// Combined propagation loss
struct propagation_params {
    double range_m;
    double frequency_hz;
    double elevation_rad      = 0.0;
    double rain_rate_mm_per_hr = 0.0;
    bool   include_atmospheric = true;
    bool   include_rain        = true;
    bool   include_lens        = true;
};

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
