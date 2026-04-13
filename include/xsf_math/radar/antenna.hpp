#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include "../core/mat3.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace xsf_math {

// Antenna gain pattern models

// Result of antenna gain evaluation
struct antenna_gain_result {
    double gain_linear = 1.0;  // linear power gain
    double gain_db     = 0.0;  // gain in dBi
    bool   in_fov      = true; // within field of view

    static antenna_gain_result from_db(double db, bool fov = true) {
        return { db_to_linear(db), db, fov };
    }
    static antenna_gain_result from_linear(double lin, bool fov = true) {
        return { lin, linear_to_db(lin), fov };
    }
};

// Isotropic antenna (0 dBi in all directions)
struct antenna_isotropic {
    antenna_gain_result evaluate(double /*az_off_rad*/, double /*el_off_rad*/) const {
        return {1.0, 0.0, true};
    }
};

// Cosine-power pattern (simple analytical model)
// G(theta) = G_peak * cos^n(theta) for |theta| < half_beamwidth
struct antenna_cosine {
    double peak_gain_db   = 30.0;   // peak gain in dBi
    double half_bw_az_rad = 5.0 * constants::deg_to_rad;  // half-power beamwidth az
    double half_bw_el_rad = 5.0 * constants::deg_to_rad;  // half-power beamwidth el
    double fov_az_rad     = 60.0 * constants::deg_to_rad;  // field of view half-angle az
    double fov_el_rad     = 60.0 * constants::deg_to_rad;  // field of view half-angle el
    double sidelobe_db    = -20.0;  // average sidelobe level

    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);

        if (!in_fov) {
            return antenna_gain_result::from_db(sidelobe_db - 10.0, false);
        }

        // Cosine rolloff from boresight
        // At half-beamwidth angle, gain drops by 3 dB
        // n = ln(0.5) / ln(cos(half_bw))
        double az_norm = (half_bw_az_rad > 1e-10) ? az_off_rad / half_bw_az_rad : 0.0;
        double el_norm = (half_bw_el_rad > 1e-10) ? el_off_rad / half_bw_el_rad : 0.0;

        double theta_norm = std::sqrt(az_norm*az_norm + el_norm*el_norm);

        double gain_db;
        if (theta_norm < 0.01) {
            gain_db = peak_gain_db;
        } else if (theta_norm <= 1.0) {
            // Main beam: Gaussian-like rolloff
            gain_db = peak_gain_db - 3.0 * theta_norm * theta_norm;
        } else {
            // Beyond main beam: transition to sidelobes
            double main_beam_edge = peak_gain_db - 3.0;
            double sll = sidelobe_db;
            double t = std::min((theta_norm - 1.0) / 2.0, 1.0);
            gain_db = lerp(main_beam_edge, sll, t);
        }

        return antenna_gain_result::from_db(gain_db, in_fov);
    }
};

// Sinc-squared pattern (typical for uniform rectangular aperture)
// G(theta) = G_peak * sinc^2(pi * D * sin(theta) / lambda)
struct antenna_sinc {
    double peak_gain_db   = 30.0;
    double aperture_az_m  = 1.0;   // aperture dimension in azimuth
    double aperture_el_m  = 1.0;   // aperture dimension in elevation
    double frequency_hz   = 10.0e9;
    double fov_az_rad     = 60.0 * constants::deg_to_rad;
    double fov_el_rad     = 60.0 * constants::deg_to_rad;

    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);
        if (!in_fov) {
            return antenna_gain_result::from_db(-40.0, false);
        }

        double lambda = constants::speed_of_light / frequency_hz;

        auto sinc = [](double x) -> double {
            if (std::abs(x) < 1e-10) return 1.0;
            double px = constants::pi * x;
            return std::sin(px) / px;
        };

        double u_az = aperture_az_m * std::sin(az_off_rad) / lambda;
        double u_el = aperture_el_m * std::sin(el_off_rad) / lambda;

        double pattern = sinc(u_az) * sinc(u_el);
        double gain_linear = db_to_linear(peak_gain_db) * pattern * pattern;

        if (gain_linear < 1e-10) gain_linear = 1e-10;
        return antenna_gain_result::from_linear(gain_linear, in_fov);
    }
};

// Tabulated antenna pattern (from measured or simulated data)
struct antenna_table {
    double peak_gain_db = 30.0;
    std::vector<double> az_angles_rad;    // sorted
    std::vector<double> el_angles_rad;    // sorted
    std::vector<std::vector<double>> pattern_db;  // [az][el], relative to peak

    double fov_az_rad = 60.0 * constants::deg_to_rad;
    double fov_el_rad = 60.0 * constants::deg_to_rad;

    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);

        double relative_db = table_lookup_2d(az_angles_rad, el_angles_rad,
                                              pattern_db, az_off_rad, el_off_rad);
        double gain_db = peak_gain_db + relative_db;

        return antenna_gain_result::from_db(gain_db, in_fov);
    }
};

// Peak gain from aperture area
inline double aperture_gain_db(double aperture_area_m2, double frequency_hz, double efficiency = 0.55) {
    double lambda = constants::speed_of_light / frequency_hz;
    double gain = efficiency * 4.0 * constants::pi * aperture_area_m2 / (lambda * lambda);
    return linear_to_db(gain);
}

// Half-power beamwidth from aperture dimension
inline double beamwidth_rad(double aperture_dim_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    return 0.886 * lambda / aperture_dim_m;  // ~51 deg * lambda/D
}

} // namespace xsf_math
