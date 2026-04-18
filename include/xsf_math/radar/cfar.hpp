#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace xsf_math {

struct cfar_detection {
    std::size_t cell_index = 0;
    double cell_power = 0.0;
    double threshold = 0.0;
};

struct cfar_result {
    std::vector<double> thresholds;
    std::vector<cfar_detection> detections;
};

struct cell_averaging_cfar {
    int guard_cells = 2;
    int training_cells = 8;
    double threshold_scale = 4.0;

    cfar_result evaluate(const std::vector<double>& power_cells) const {
        cfar_result out;
        out.thresholds.resize(power_cells.size(), 0.0);
        if (power_cells.empty()) return out;

        int window = guard_cells + training_cells;
        for (std::size_t i = 0; i < power_cells.size(); ++i) {
            if (static_cast<int>(i) < window ||
                static_cast<int>(i) + window >= static_cast<int>(power_cells.size())) {
                continue;
            }

            double sum = 0.0;
            int count = 0;
            for (int k = -window; k <= window; ++k) {
                if (std::abs(k) <= guard_cells) continue;
                sum += power_cells[static_cast<std::size_t>(static_cast<int>(i) + k)];
                ++count;
            }
            if (count <= 0) continue;

            double noise = sum / static_cast<double>(count);
            double threshold = noise * threshold_scale;
            out.thresholds[i] = threshold;
            if (power_cells[i] > threshold) {
                out.detections.push_back({i, power_cells[i], threshold});
            }
        }
        return out;
    }
};

struct doppler_ambiguity_result {
    double unambiguous_velocity_mps = 0.0;
    double blind_speed_mps = 0.0;
    double folded_velocity_mps = 0.0;
    double folded_frequency_hz = 0.0;
    int ambiguity_index = 0;
};

inline doppler_ambiguity_result evaluate_doppler_ambiguity(double radial_velocity_mps,
                                                           double carrier_frequency_hz,
                                                           double prf_hz) {
    doppler_ambiguity_result out;
    if (carrier_frequency_hz <= 0.0 || prf_hz <= 0.0) return out;

    double wavelength = constants::speed_of_light / carrier_frequency_hz;
    double doppler_hz = 2.0 * radial_velocity_mps / wavelength;
    double folded = std::remainder(doppler_hz, prf_hz);

    out.folded_frequency_hz = folded;
    out.folded_velocity_mps = 0.5 * folded * wavelength;
    out.unambiguous_velocity_mps = 0.25 * prf_hz * wavelength;
    out.blind_speed_mps = 0.5 * prf_hz * wavelength;
    out.ambiguity_index = static_cast<int>(std::llround((doppler_hz - folded) / prf_hz));
    return out;
}

}  // namespace xsf_math
