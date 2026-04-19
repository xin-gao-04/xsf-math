#pragma once

#include <cmath>
#include <limits>

namespace xsf_math {

/** 数学与物理常数集合 (Collection of mathematical and physical constants) */
struct constants {
    static constexpr double pi            = 3.14159265358979323846; ///< 圆周率 (pi)
    static constexpr double two_pi        = 2.0 * pi;               ///< 2π (two pi)
    static constexpr double half_pi       = pi / 2.0;               ///< π/2 (half pi)
    static constexpr double deg_to_rad    = pi / 180.0;             ///< 度转弧度 (degrees to radians)
    static constexpr double rad_to_deg    = 180.0 / pi;             ///< 弧度转度 (radians to degrees)

    // 物理常数
    static constexpr double speed_of_light  = 299792458.0;        ///< 光速 (speed of light)，单位 m/s
    static constexpr double boltzmann_k     = 1.380649e-23;       ///< 玻尔兹曼常数 (Boltzmann constant)，单位 J/K
    static constexpr double earth_radius_m  = 6371000.0;          ///< 地球平均半径 (mean Earth radius)，单位 m（平均值）
    static constexpr double earth_equatorial_radius_m = 6378137.0; ///< WGS-84 长半轴 (WGS-84 equatorial radius)，单位 m
    static constexpr double earth_flattening = 1.0 / 298.257223563; ///< 地球扁率 (Earth flattening)
    static constexpr double earth_eccentricity_sq =
        earth_flattening * (2.0 - earth_flattening);              ///< 地球第一偏心率平方 (Earth eccentricity squared)
    static constexpr double earth_rotation_rate_rad_s = 7.2921150e-5; ///< 地球自转角速度 (Earth rotation rate)，单位 rad/s
    static constexpr double julian_date_j2000 = 2451545.0;        ///< J2000 历元儒略日 (Julian date of J2000 epoch)
    static constexpr double gravity_mps2    = 9.80665;            ///< 标准重力加速度 (standard gravity)，单位 m/s²

    // 大气（ISA 海平面）
    static constexpr double ssl_temperature = 288.15;             ///< 海平面标准温度 (sea-level standard temperature)，单位 K（15 °C）
    static constexpr double ssl_pressure    = 101325.0;           ///< 海平面标准压强 (sea-level standard pressure)，单位 Pa
    static constexpr double ssl_density     = 1.225;              ///< 海平面标准密度 (sea-level standard density)，单位 kg/m³
    static constexpr double ssl_sonic_vel   = 340.294;            ///< 海平面声速 (sea-level speed of sound)，单位 m/s
    static constexpr double lapse_rate      = 0.0065;             ///< 温度直减率 (temperature lapse rate)，单位 K/m（对流层）
    static constexpr double tropopause_alt  = 11000.0;            ///< 对流层顶高度 (tropopause altitude)，单位 m
    static constexpr double tropopause_temp = 216.65;             ///< 对流层顶温度 (tropopause temperature)，单位 K
    static constexpr double gamma_air       = 1.4;                ///< 空气比热比 (ratio of specific heats for air)
    static constexpr double gas_constant_air = 287.058;           ///< 空气气体常数 (specific gas constant for air)，单位 J/(kg·K)
    static constexpr double gmr             = 0.0341631947;       ///< 重力与摩尔气体常数之比 g·M/R (gravitational constant over molar gas constant)，单位 1/m

    // 单位换算
    static constexpr double ft_to_m         = 0.3048;             ///< 英尺转米 (feet to meters)
    static constexpr double m_to_ft         = 1.0 / 0.3048;       ///< 米转英尺 (meters to feet)
    static constexpr double nm_to_m         = 1852.0;             ///< 海里转米 (nautical miles to meters)
    static constexpr double kt_to_mps       = 1852.0 / 3600.0;   ///< 节转米每秒 (knots to meters per second)
    static constexpr double mps_to_kt       = 3600.0 / 1852.0;   ///< 米每秒转节 (meters per second to knots)
    static constexpr double dbw_to_w        = 1.0;                ///< dBW 转瓦特占位值 (placeholder for dBW to watts)
};

/** dB 转线性功率比 (Convert dB to linear power ratio) */
inline double db_to_linear(double db) noexcept {
    return std::pow(10.0, db / 10.0);
}

/** 线性功率比转 dB (Convert linear power ratio to dB) */
inline double linear_to_db(double linear) noexcept {
    return 10.0 * std::log10(linear);
}

/** dB 转线性功率比（同 db_to_linear） (Convert dB to linear power ratio, same as db_to_linear) */
inline double db_to_linear_power(double db) noexcept {
    return db_to_linear(db);
}

/** dB 转线性电压比 (Convert dB to linear voltage ratio) */
inline double db_to_linear_voltage(double db) noexcept {
    return std::pow(10.0, db / 20.0);
}

} // namespace xsf_math
