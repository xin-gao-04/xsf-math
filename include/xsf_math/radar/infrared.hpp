#pragma once

#include "../radar/marcum_swerling.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct infrared_target_signature {
    double projected_area_m2 = 1.0;
    double temperature_k = 500.0;
    double emissivity = 0.9;

    double radiant_power_w() const {
        constexpr double stefan_boltzmann = 5.670374419e-8;
        return emissivity * stefan_boltzmann * projected_area_m2 *
               std::pow(std::max(temperature_k, 1.0), 4.0);
    }
};

struct infrared_atmosphere {
    double extinction_per_km = 0.06;

    double transmittance(double range_m) const {
        return std::exp(-extinction_per_km * (range_m / 1000.0));
    }
};

struct infrared_detection_result {
    double emitted_power_w = 0.0;
    double received_power_w = 0.0;
    double snr_linear = 0.0;
    double pd = 0.0;
    bool detected = false;
};

struct infrared_detector {
    double aperture_area_m2 = 0.015;
    double optics_transmission = 0.75;
    double noise_equivalent_power_w = 1.0e-11;
    double required_pd = 0.5;
    marcum_swerling detector_model{};

    infrared_detection_result evaluate(const infrared_target_signature& target,
                                       const infrared_atmosphere& atmosphere,
                                       double range_m) const {
        infrared_detection_result out;
        out.emitted_power_w = target.radiant_power_w();
        double spreading = 4.0 * constants::pi * std::max(range_m * range_m, 1.0);
        out.received_power_w = out.emitted_power_w * aperture_area_m2 * optics_transmission *
                               atmosphere.transmittance(range_m) / spreading;
        out.snr_linear = out.received_power_w / std::max(noise_equivalent_power_w, 1.0e-20);
        out.pd = detector_model.compute_pd(out.snr_linear);
        out.detected = out.pd >= required_pd;
        return out;
    }
};

}  // namespace xsf_math
