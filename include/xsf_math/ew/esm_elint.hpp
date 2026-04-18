#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct emitter_signal {
    double incident_power_w = 0.0;
    double carrier_frequency_hz = 0.0;
    double prf_hz = 0.0;
    double pulse_width_s = 0.0;
    double scan_period_s = 0.0;
};

struct esm_contact {
    bool intercepted = false;
    bool classified = false;
    double intercept_probability = 0.0;
    double classification_probability = 0.0;
    double pulse_density_hz = 0.0;
    double bearing_accuracy_rad = constants::half_pi;
};

struct esm_receiver {
    double sensitivity_w = 1.0e-12;
    double classification_threshold_w = 5.0e-12;
    int pulses_for_classification = 32;
    double scan_alignment_factor = 0.8;

    double intercept_probability(const emitter_signal& signal, double dwell_time_s) const {
        double power_ratio = signal.incident_power_w / std::max(sensitivity_w, 1.0e-20);
        double pulses = std::max(signal.prf_hz * dwell_time_s, 0.0);
        double pulse_term = 1.0 - std::exp(-pulses / 8.0);
        double power_term = 1.0 - std::exp(-power_ratio);
        return std::clamp(scan_alignment_factor * pulse_term * power_term, 0.0, 1.0);
    }

    esm_contact evaluate(const emitter_signal& signal, double dwell_time_s) const {
        esm_contact out;
        out.intercept_probability = intercept_probability(signal, dwell_time_s);
        out.intercepted = out.intercept_probability > 0.2;
        out.pulse_density_hz = signal.prf_hz;

        double class_power_ratio =
            signal.incident_power_w / std::max(classification_threshold_w, 1.0e-20);
        double pulses = std::max(signal.prf_hz * dwell_time_s, 0.0);
        double pulse_ratio = pulses / std::max(static_cast<double>(pulses_for_classification), 1.0);
        out.classification_probability =
            std::clamp((1.0 - std::exp(-class_power_ratio)) * std::min(pulse_ratio, 1.0), 0.0, 1.0);
        out.classified = out.classification_probability > 0.5;

        if (out.intercepted) {
            double snr = std::max(signal.incident_power_w / std::max(sensitivity_w, 1.0e-20), 1.0);
            out.bearing_accuracy_rad = std::clamp(0.35 / std::sqrt(snr), 0.5 * constants::deg_to_rad,
                                                  45.0 * constants::deg_to_rad);
        }
        return out;
    }
};

}  // namespace xsf_math
