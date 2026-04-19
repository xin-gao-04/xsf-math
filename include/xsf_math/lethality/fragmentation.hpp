#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 破片飞散模式参数（Fragment dispersal pattern parameters）
struct fragment_pattern {
    int fragment_count = 2000;                // 破片数量（Fragment count）
    double mean_fragment_mass_kg = 0.01;      // 平均破片质量（Mean fragment mass），单位 kg
    double initial_velocity_mps = 1800.0;     // 初始飞散速度（Initial velocity），单位 m/s
    double cone_angle_rad = constants::pi;    // 飞散锥角（Cone angle），单位 rad；pi 表示全向飞散
};

// 破片云杀伤效应结果（Fragment cloud effect result）
struct fragment_effect_result {
    double cloud_area_m2 = 0.0;           // 破片云覆盖面积（Cloud area），单位 m^2
    double areal_density_per_m2 = 0.0;    // 面密度（Areal density），单位 个/m^2
    double mean_fragment_energy_j = 0.0;  // 平均破片动能（Mean fragment energy），单位 J
    double hit_probability = 0.0;         // 命中概率（Hit probability），范围 [0,1]
};

// 格尼公式（Gurney equation）计算破片初速（Calculate fragment velocity using Gurney equation）
// explosive_mass_kg: 装药质量（kg）; casing_mass_kg: 壳体质量（kg）
// gurney_constant_mps: 格尼常数（m/s），典型值约 2400 m/s
inline double gurney_fragment_velocity(double explosive_mass_kg,
                                       double casing_mass_kg,
                                       double gurney_constant_mps = 2400.0) {
    if (explosive_mass_kg <= 0.0 || casing_mass_kg <= 0.0) return 0.0;
    return gurney_constant_mps * std::sqrt(2.0 * explosive_mass_kg / casing_mass_kg);
}

// 评估破片云杀伤效应（Evaluate fragment cloud lethality）
// pattern: 破片模式; range_m: 距离（m）; presented_area_m2: 目标投影面积（m^2）
inline fragment_effect_result evaluate_fragment_cloud(const fragment_pattern& pattern,
                                                      double range_m,
                                                      double presented_area_m2) {
    fragment_effect_result out;
    // 立体角（Solid angle）计算
    double solid_angle = 2.0 * constants::pi * (1.0 - std::cos(0.5 * pattern.cone_angle_rad));
    out.cloud_area_m2 = std::max(solid_angle * range_m * range_m, 1.0);
    out.areal_density_per_m2 = pattern.fragment_count / out.cloud_area_m2;
    out.mean_fragment_energy_j =
        0.5 * pattern.mean_fragment_mass_kg * pattern.initial_velocity_mps * pattern.initial_velocity_mps;
    // 泊松命中概率模型（Poisson hit probability model）
    out.hit_probability = 1.0 - std::exp(-out.areal_density_per_m2 * std::max(presented_area_m2, 0.0));
    return out;
}

}  // namespace xsf_math
