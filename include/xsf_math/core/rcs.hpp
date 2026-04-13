#pragma once

#include "constants.hpp"
#include "interpolation.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 雷达散射截面（RCS）查表与建模
// 支持频率相关、姿态角相关和 Swerling 起伏模型

// 以 m^2 或 dBsm 表示的 RCS 值
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

// 简单的恒定 RCS 模型
struct rcs_constant {
    double sigma_m2;

    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double /*freq_hz*/) const {
        return rcs_value::from_m2(sigma_m2);
    }
};

// 姿态角相关 RCS 表（仅方位角，单频）
struct rcs_table_1d {
    std::vector<double> azimuths_rad;  // 升序排列 [0, 2*pi]
    std::vector<double> sigma_dbsm;    // 对应的 RCS 值

    rcs_value evaluate(double az_rad, double /*el_rad*/ = 0.0) const {
        // 归一化到 [0, 2pi]
        double a = std::fmod(az_rad, constants::two_pi);
        if (a < 0) a += constants::two_pi;
        double dbsm = table_lookup(azimuths_rad, sigma_dbsm, a);
        return rcs_value::from_dbsm(dbsm);
    }
};

// 二维 RCS 表：方位角 x 俯仰角
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

// 频率相关 RCS：按频率选择表
struct rcs_freq_dependent {
    struct freq_entry {
        double freq_hz;
        rcs_table_1d table;
    };
    std::vector<freq_entry> entries;  // 按 freq_hz 升序排列

    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        if (entries.empty()) return rcs_value::from_m2(0.0);
        if (entries.size() == 1) return entries[0].table.evaluate(az_rad, el_rad);

        // 找到夹逼频点并在 dBsm 域插值
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

// 使用角平分线近似双基地 RCS
// sigma_bi ≈ sigma_mono(bisector_angle)
// bisector = normalize(unit_tx + unit_rx)
inline double bistatic_bisector_angle(const vec3& unit_to_tx, const vec3& unit_to_rx) {
    vec3 bisector = (unit_to_tx + unit_to_rx).normalized();
    return std::acos(clamp(bisector.x, -1.0, 1.0));  // 相对前向（x 轴）的夹角
}

// RCS 频率区间分类
enum class rcs_regime {
    rayleigh,      // 目标尺寸 << 波长
    mie,           // 目标尺寸 ~ 波长（共振）
    optical        // 目标尺寸 >> 波长
};

inline rcs_regime classify_regime(double target_size_m, double freq_hz) {
    double wavelength = constants::speed_of_light / freq_hz;
    double ratio = target_size_m / wavelength;
    if (ratio < 0.1)  return rcs_regime::rayleigh;
    if (ratio < 5.0)  return rcs_regime::mie;
    return rcs_regime::optical;
}

// Swerling 起伏模型
// 根据 Swerling 方案对平均 RCS 施加统计起伏
enum class swerling_case { case0 = 0, case1 = 1, case2 = 2, case3 = 3, case4 = 4 };

} // namespace xsf_math
