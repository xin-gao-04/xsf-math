#pragma once

#include "constants.hpp"
#include "interpolation.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Radar Cross Section (RCS) lookup and modeling
// Supports frequency-dependent, aspect-angle-dependent, and Swerling fluctuation models

// RCS value in m^2 or dBsm
struct rcs_value {
    double m2   = 0.0;
    double dbsm = -999.0;

    static rcs_value from_m2(double v) {
        return { v, (v > 0) ? 10.0 * std::log10(v) : -999.0 };
    }
    static rcs_value from_dbsm(double v) {
        return { std::pow(10.0, v / 10.0), v };
    }
};

// Simple constant RCS model
struct rcs_constant {
    double sigma_m2;

    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double /*freq_hz*/) const {
        return rcs_value::from_m2(sigma_m2);
    }
};

// Aspect-angle dependent RCS table (azimuth only, single frequency)
struct rcs_table_1d {
    std::vector<double> azimuths_rad;  // sorted ascending [0, 2*pi]
    std::vector<double> sigma_dbsm;    // corresponding RCS values

    rcs_value evaluate(double az_rad, double /*el_rad*/ = 0.0) const {
        // Wrap to [0, 2pi]
        double a = std::fmod(az_rad, constants::two_pi);
        if (a < 0) a += constants::two_pi;
        double dbsm = table_lookup(azimuths_rad, sigma_dbsm, a);
        return rcs_value::from_dbsm(dbsm);
    }
};

// 2D RCS table: azimuth x elevation
struct rcs_table_2d {
    std::vector<double> azimuths_rad;
    std::vector<double> elevations_rad;
    std::vector<std::vector<double>> sigma_dbsm;  // [az_idx][el_idx]

    rcs_value evaluate(double az_rad, double el_rad) const {
        double a = std::fmod(az_rad, constants::two_pi);
        if (a < 0) a += constants::two_pi;
        double dbsm = table_lookup_2d(azimuths_rad, elevations_rad, sigma_dbsm, a, el_rad);
        return rcs_value::from_dbsm(dbsm);
    }
};

// Frequency-dependent RCS: select table based on frequency
struct rcs_freq_dependent {
    struct freq_entry {
        double freq_hz;
        rcs_table_1d table;
    };
    std::vector<freq_entry> entries;  // sorted by freq_hz ascending

    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        if (entries.empty()) return rcs_value::from_m2(0.0);
        if (entries.size() == 1) return entries[0].table.evaluate(az_rad, el_rad);

        // Find bracketing frequencies and interpolate in dBsm
        if (freq_hz <= entries.front().freq_hz)
            return entries.front().table.evaluate(az_rad, el_rad);
        if (freq_hz >= entries.back().freq_hz)
            return entries.back().table.evaluate(az_rad, el_rad);

        size_t i = 0;
        for (; i < entries.size() - 1; ++i) {
            if (freq_hz < entries[i+1].freq_hz) break;
        }
        double t = (freq_hz - entries[i].freq_hz) / (entries[i+1].freq_hz - entries[i].freq_hz);
        double dbsm_lo = entries[i].table.evaluate(az_rad, el_rad).dbsm;
        double dbsm_hi = entries[i+1].table.evaluate(az_rad, el_rad).dbsm;
        return rcs_value::from_dbsm(lerp(dbsm_lo, dbsm_hi, t));
    }
};

// Bistatic RCS approximation using bisector angle
// sigma_bi ≈ sigma_mono(bisector_angle)
// bisector = normalize(unit_tx + unit_rx)
inline double bistatic_bisector_angle(const vec3& unit_to_tx, const vec3& unit_to_rx) {
    vec3 bisector = (unit_to_tx + unit_to_rx).normalized();
    return std::acos(clamp(bisector.x, -1.0, 1.0));  // angle from forward (x)
}

// RCS frequency regime classification
enum class rcs_regime {
    rayleigh,      // target << wavelength
    mie,           // target ~ wavelength (resonance)
    optical        // target >> wavelength
};

inline rcs_regime classify_regime(double target_size_m, double freq_hz) {
    double wavelength = constants::speed_of_light / freq_hz;
    double ratio = target_size_m / wavelength;
    if (ratio < 0.1)  return rcs_regime::rayleigh;
    if (ratio < 5.0)  return rcs_regime::mie;
    return rcs_regime::optical;
}

// Swerling fluctuation model
// Applies statistical fluctuation to a mean RCS based on Swerling case
enum class swerling_case { case0 = 0, case1 = 1, case2 = 2, case3 = 3, case4 = 4 };

} // namespace xsf_math
