#pragma once

#include "constants.hpp"
#include "interpolation.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace xsf_math {

/** 雷达散射截面（RCS）查表与建模 (Radar Cross Section (RCS) lookup and modeling) */
// 支持频率相关、姿态角相关和 Swerling 起伏模型
// Supports frequency-dependent, aspect-angle-dependent, and Swerling fluctuation models

/** 以 m² 或 dBsm 表示的 RCS 值 (RCS value expressed in m² or dBsm) */
struct rcs_value {
    double m2   = 0.0;   ///< RCS 线性值 (RCS linear value)，单位 m²
    double dbsm = -999.0; ///< RCS 对数值 (RCS logarithmic value)，单位 dBsm

    /** 由线性值 m² 构造 (Construct from linear value in m²) */
    static rcs_value from_m2(double v) {
        return { v, (v > 0) ? 10.0 * std::log10(v) : -999.0 };
    }

    /** 由对数值 dBsm 构造 (Construct from logarithmic value in dBsm) */
    static rcs_value from_dbsm(double v) {
        return { std::pow(10.0, v / 10.0), v };
    }
};

/** 简单的恒定 RCS 模型 (Simple constant RCS model) */
struct rcs_constant {
    double sigma_m2; ///< 恒定 RCS 值 (constant RCS value)，单位 m²

    /** 评估 RCS（与姿态、频率无关） (Evaluate RCS, independent of aspect and frequency) */
    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double /*freq_hz*/) const {
        return rcs_value::from_m2(sigma_m2);
    }
};

/** 姿态角相关 RCS 表（仅方位角，单频） (Aspect-angle-dependent RCS table, azimuth only, single frequency) */
struct rcs_table_1d {
    std::vector<double> azimuths_rad; ///< 方位角采样数组 (azimuth sample array)，升序排列 [0, 2π]
    std::vector<double> sigma_dbsm;   ///< 对应 RCS 值 (corresponding RCS values)，单位 dBsm

    /** 按方位角查找并插值 RCS (Lookup and interpolate RCS by azimuth) */
    rcs_value evaluate(double az_rad, double /*el_rad*/ = 0.0) const {
        // 归一化到 [0, 2π] (Normalize to [0, 2π])
        double a = std::fmod(az_rad, constants::two_pi);
        if (a < 0) a += constants::two_pi;
        double dbsm = table_lookup(azimuths_rad, sigma_dbsm, a);
        return rcs_value::from_dbsm(dbsm);
    }
};

/** 二维 RCS 表：方位角 × 俯仰角 (2D RCS table: azimuth × elevation) */
struct rcs_table_2d {
    std::vector<double> azimuths_rad; ///< 方位角采样数组 (azimuth sample array)
    std::vector<double> elevations_rad; ///< 俯仰角采样数组 (elevation sample array)
    std::vector<std::vector<double>> sigma_dbsm; ///< RCS 值矩阵 [az_idx][el_idx] (RCS value matrix)

    /** 按方位角和俯仰角双线性插值查找 RCS (Lookup RCS by azimuth and elevation with bilinear interpolation) */
    rcs_value evaluate(double az_rad, double el_rad) const {
        double a = std::fmod(az_rad, constants::two_pi);
        if (a < 0) a += constants::two_pi;
        double dbsm = table_lookup_2d(azimuths_rad, elevations_rad, sigma_dbsm, a, el_rad);
        return rcs_value::from_dbsm(dbsm);
    }
};

/** 频率相关 RCS：按频率选择表 (Frequency-dependent RCS: select table by frequency) */
struct rcs_freq_dependent {
    /** 频率-表条目 (Frequency-table entry) */
    struct freq_entry {
        double freq_hz;   ///< 频率 (frequency)，单位 Hz
        rcs_table_1d table; ///< 对应方位角 RCS 表 (corresponding azimuth RCS table)
    };
    std::vector<freq_entry> entries; ///< 频率条目数组，按 freq_hz 升序排列 (frequency entry array, sorted ascending by freq_hz)

    /** 按方位角、俯仰角和频率评估 RCS，在 dBsm 域插值 (Evaluate RCS by azimuth, elevation, and frequency, interpolating in dBsm domain) */
    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        if (entries.empty()) return rcs_value::from_m2(0.0);
        if (entries.size() == 1) return entries[0].table.evaluate(az_rad, el_rad);

        // 找到夹逼频点并在 dBsm 域插值 (Find bracketing frequencies and interpolate in dBsm domain)
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

/** 使用角平分线近似双基地 RCS (Approximate bistatic RCS using bisector angle) */
// sigma_bi ≈ sigma_mono(bisector_angle)
// bisector = normalize(unit_tx + unit_rx)
inline double bistatic_bisector_angle(const vec3& unit_to_tx, const vec3& unit_to_rx) {
    vec3 bisector = (unit_to_tx + unit_to_rx).normalized();
    return std::acos(clamp(bisector.x, -1.0, 1.0));  ///< 相对前向（x 轴）的夹角 (angle relative to forward x-axis)
}

/** RCS 频率区间分类 (RCS frequency regime classification) */
enum class rcs_regime {
    rayleigh,      ///< 瑞利区：目标尺寸 << 波长 (Rayleigh region: target size << wavelength)
    mie,           ///< 谐振区（Mie 区）：目标尺寸 ~ 波长 (Mie/resonance region: target size ~ wavelength)
    optical        ///< 光学区：目标尺寸 >> 波长 (Optical region: target size >> wavelength)
};

/** 根据目标尺寸与波长之比分类 RCS 区间 (Classify RCS regime by ratio of target size to wavelength) */
inline rcs_regime classify_regime(double target_size_m, double freq_hz) {
    double wavelength = constants::speed_of_light / freq_hz;
    double ratio = target_size_m / wavelength;
    if (ratio < 0.1)  return rcs_regime::rayleigh;
    if (ratio < 5.0)  return rcs_regime::mie;
    return rcs_regime::optical;
}

/** Swerling 起伏模型枚举 (Swerling fluctuation model enumeration) */
// 根据 Swerling 方案对平均 RCS 施加统计起伏
// Apply statistical fluctuation to mean RCS according to Swerling model
enum class swerling_case { case0 = 0, case1 = 1, case2 = 2, case3 = 3, case4 = 4 };

} // namespace xsf_math
