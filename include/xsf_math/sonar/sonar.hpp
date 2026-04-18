#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct sonar_environment {
    double absorption_db_per_km = 0.8;
    double ambient_noise_db = 60.0;
    double reverberation_db = 55.0;
};

struct sonar_contact {
    double transmission_loss_db = 0.0;
    double snr_db = 0.0;
    double pd = 0.0;
    bool detected = false;
};

inline double sonar_transmission_loss_db(double range_m, double absorption_db_per_km) {
    double range_km = std::max(range_m / 1000.0, 1.0e-6);
    return 20.0 * std::log10(range_km * 1000.0) + absorption_db_per_km * range_km;
}

struct active_sonar_equation {
    sonar_environment environment{};
    double source_level_db = 215.0;
    double target_strength_db = 10.0;
    double detection_threshold_db = 10.0;

    sonar_contact evaluate(double range_m) const {
        sonar_contact out;
        out.transmission_loss_db = sonar_transmission_loss_db(range_m, environment.absorption_db_per_km);
        double echo_level = source_level_db - 2.0 * out.transmission_loss_db + target_strength_db;
        double interference = std::max(environment.ambient_noise_db, environment.reverberation_db);
        out.snr_db = echo_level - interference;
        out.pd = std::clamp((out.snr_db - detection_threshold_db + 12.0) / 24.0, 0.0, 1.0);
        out.detected = out.snr_db >= detection_threshold_db;
        return out;
    }
};

struct passive_sonar_equation {
    sonar_environment environment{};
    double source_level_db = 165.0;
    double array_gain_db = 12.0;
    double detection_threshold_db = 3.0;

    sonar_contact evaluate(double range_m) const {
        sonar_contact out;
        out.transmission_loss_db = sonar_transmission_loss_db(range_m, environment.absorption_db_per_km);
        double received_level = source_level_db - out.transmission_loss_db + array_gain_db;
        out.snr_db = received_level - environment.ambient_noise_db;
        out.pd = std::clamp((out.snr_db - detection_threshold_db + 10.0) / 20.0, 0.0, 1.0);
        out.detected = out.snr_db >= detection_threshold_db;
        return out;
    }
};

}  // namespace xsf_math
