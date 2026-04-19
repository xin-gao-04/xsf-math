#pragma once

#include "../core/constants.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

namespace xsf_math {

// 杀伤概率（Probability of Kill, Pk）模型 (Kill probability models)

// Pk 随脱靶量变化的分级杀伤曲线（Pk curve as a function of miss distance）
struct pk_curve {
    std::vector<double> miss_distance_m;  // 脱靶量（Miss distance），升序排列，单位 m
    std::vector<double> pk_values;        // 对应的 Pk 值（Kill probability values），范围 [0,1]

    // 根据脱靶量评估 Pk（Evaluate Pk for a given miss distance）
    double evaluate(double miss_dist_m) const {
        if (miss_distance_m.empty()) return 0.0;
        return table_lookup(miss_distance_m, pk_values, std::abs(miss_dist_m));
    }

    // 生成典型的爆破破片战斗部杀伤曲线（Generate typical blast-fragmentation warhead curve）
    // lethal_radius_m: 杀伤半径（m）; max_radius_m: 最大计算半径（m）
    static pk_curve blast_fragmentation(double lethal_radius_m, double max_radius_m) {
        pk_curve curve;
        int n = 20;
        for (int i = 0; i <= n; ++i) {
            double d = max_radius_m * static_cast<double>(i) / n;
            curve.miss_distance_m.push_back(d);

            double pk;
            if (d <= lethal_radius_m * 0.5) {
                pk = 1.0;  // 在半个杀伤半径内必杀（Certain kill within half lethal radius）
            } else if (d <= lethal_radius_m) {
                // 平滑衰减（Smooth decay）
                double t = (d - lethal_radius_m * 0.5) / (lethal_radius_m * 0.5);
                pk = 1.0 - 0.3 * t * t;
            } else {
                // 超出杀伤半径后指数衰减（Exponential decay beyond lethal radius）
                double excess = (d - lethal_radius_m) / lethal_radius_m;
                pk = 0.7 * std::exp(-2.0 * excess * excess);
            }
            curve.pk_values.push_back(std::max(pk, 0.0));
        }
        return curve;
    }

    // 生成连续杆战斗部杀伤曲线（Generate continuous-rod warhead curve）
    // rod_radius_m: 杆条展开半径（m）
    static pk_curve continuous_rod(double rod_radius_m) {
        pk_curve curve;
        curve.miss_distance_m = {0.0, rod_radius_m * 0.5, rod_radius_m, rod_radius_m * 1.5, rod_radius_m * 3.0};
        curve.pk_values       = {1.0, 0.95,                0.6,          0.1,                  0.0};
        return curve;
    }
};

// 使用脱靶量平方进行杀伤评估（Kill assessment using squared miss distance, avoids square root）
struct kill_assessment {
    double lethal_radius_sq_m2;  // 杀伤半径平方（Lethal radius squared），单位 m^2

    // 快速检查：脱靶量是否落在杀伤包线内？（Quick check: is miss distance within lethal envelope?）
    bool within_lethal_envelope(double miss_dist_sq_m2) const {
        return miss_dist_sq_m2 <= lethal_radius_sq_m2;
    }
};

// 单发 Pk（Single Shot Probability of Kill, SSPK）计算（Calculate single-shot Pk）
inline double single_shot_pk(double miss_distance_m,   // 脱靶量（m）
                              const pk_curve& curve) {
    return curve.evaluate(miss_distance_m);
}

// N 次独立射击的累计 Pk（Cumulative Pk for N independent shots）
inline double cumulative_pk(const std::vector<double>& individual_pks) {
    double p_survive = 1.0;
    for (double pk : individual_pks) {
        p_survive *= (1.0 - pk);
    }
    return 1.0 - p_survive;
}

// 蒙特卡洛杀伤判定（Monte Carlo kill adjudication）
struct monte_carlo_kill {
    std::mt19937 rng;  // 随机数生成器（Random number generator）

    explicit monte_carlo_kill(unsigned seed = 42) : rng(seed) {}

    // 若目标被杀伤则返回 true（Return true if target is killed）
    // pk: 杀伤概率（Kill probability）
    bool evaluate(double pk) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(rng) < pk;
    }

    // 使用脱靶量和 Pk 曲线进行评估（Evaluate using miss distance and Pk curve）
    bool evaluate(double miss_distance_m, const pk_curve& curve) {
        double pk = curve.evaluate(miss_distance_m);
        return evaluate(pk);
    }
};

// 目标脆弱性分类（Target vulnerability classification）
enum class target_class {
    fighter,           // 战斗机（Fighter）
    bomber,            // 轰炸机（Bomber）
    transport,         // 运输机（Transport）
    cruise_missile,    // 巡航导弹（Cruise missile）
    ballistic_missile, // 弹道导弹（Ballistic missile）
    helicopter,        // 直升机（Helicopter）
    uav,               // 无人机（Unmanned Aerial Vehicle, UAV）
    ground_vehicle,    // 地面车辆（Ground vehicle）
    ship               // 舰船（Ship）
};

// 各目标类型的典型杀伤半径（Typical lethal radius for each target type），单位 m
inline double typical_lethal_radius(target_class tc) {
    switch (tc) {
    case target_class::fighter:           return 8.0;
    case target_class::bomber:            return 12.0;
    case target_class::transport:         return 15.0;
    case target_class::cruise_missile:    return 5.0;
    case target_class::ballistic_missile: return 10.0;
    case target_class::helicopter:        return 10.0;
    case target_class::uav:              return 6.0;
    case target_class::ground_vehicle:    return 3.0;
    case target_class::ship:              return 20.0;
    default: return 10.0;
    }
}

// 电子战对 Pk 的降级（EW degradation effect on Pk）
// 干扰会降低跟踪精度 -> 增大脱靶量（Jamming reduces tracking accuracy -> increased miss distance）
inline double ew_degraded_miss_distance(double baseline_miss_m,         // 基线脱靶量（Baseline miss distance，m）
                                         double track_error_increase_m) {// 跟踪误差增量（Track error increase，m）
    return std::sqrt(baseline_miss_m * baseline_miss_m +
                     track_error_increase_m * track_error_increase_m);
}

} // namespace xsf_math
