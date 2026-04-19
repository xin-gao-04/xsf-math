#pragma once

#include "../radar/marcum_swerling.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 红外目标特性 (Infrared target signature)
struct infrared_target_signature {
    double projected_area_m2 = 1.0;  // 投影面积（m^2）(Projected area)
    double temperature_k = 500.0;    // 表面温度（K）(Surface temperature)
    double emissivity = 0.9;         // 发射率 (Emissivity)

    // 辐射功率（W）(Radiant power in Watts)
    double radiant_power_w() const {
        constexpr double stefan_boltzmann = 5.670374419e-8; // 斯特藩-玻尔兹曼常数 (Stefan-Boltzmann constant)
        return emissivity * stefan_boltzmann * projected_area_m2 *
               std::pow(std::max(temperature_k, 1.0), 4.0);
    }
};

// 红外大气传输特性 (Infrared atmospheric transmission)
struct infrared_atmosphere {
    double extinction_per_km = 0.06; // 消光系数（每 km）(Extinction coefficient per km)

    // 大气透过率 (Atmospheric transmittance)
    double transmittance(double range_m) const {
        return std::exp(-extinction_per_km * (range_m / 1000.0));
    }
};

// 红外探测结果 (Infrared detection result)
struct infrared_detection_result {
    double emitted_power_w = 0.0;  // 目标辐射功率（W）(Emitted power)
    double received_power_w = 0.0; // 探测器接收功率（W）(Received power)
    double snr_linear = 0.0;       // 信噪比（线性值）(SNR linear)
    double pd = 0.0;               // 探测概率 (Probability of detection)
    bool detected = false;         // 是否检测到 (Detection flag)
};

// 红外探测器模型 (Infrared detector model)
struct infrared_detector {
    double aperture_area_m2 = 0.015;          // 光学孔径面积（m^2）(Optical aperture area)
    double optics_transmission = 0.75;        // 光学系统透过率 (Optics transmission)
    double noise_equivalent_power_w = 1.0e-11; // 噪声等效功率（W）(Noise equivalent power, NEP)
    double required_pd = 0.5;                 // 要求探测概率 (Required probability of detection)
    marcum_swerling detector_model{};         // 探测统计模型 (Detection statistical model)

    // 评估对给定目标在指定距离上的探测性能 (Evaluate detection performance for target at given range)
    infrared_detection_result evaluate(const infrared_target_signature& target,
                                       const infrared_atmosphere& atmosphere,
                                       double range_m) const {
        infrared_detection_result out;
        out.emitted_power_w = target.radiant_power_w();
        // 球面扩散损耗：4*pi*R^2 (Spherical spreading loss)
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
