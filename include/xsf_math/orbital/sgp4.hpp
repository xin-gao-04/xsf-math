#pragma once

#include "../core/constants.hpp"
#include "j2.hpp"
#include "kepler.hpp"
#include <cmath>
#include <cstdlib>
#include <string>

namespace xsf_math {

// 两行根数（Two-Line Element, TLE）记录结构 (TLE record structure)
struct tle_record {
    std::string name;                    // 卫星名称（Satellite name）
    double inclination_rad = 0.0;        // 轨道倾角（Inclination），单位 rad
    double raan_rad = 0.0;               // 升交点赤经（RAAN），单位 rad
    double eccentricity = 0.0;           // 偏心率（Eccentricity）
    double arg_perigee_rad = 0.0;        // 近地点幅角（Argument of perigee），单位 rad
    double mean_anomaly_rad = 0.0;       // 平近点角（Mean anomaly），单位 rad
    double mean_motion_rev_per_day = 0.0;// 平均运动（Mean motion），单位 转/天（revolutions/day）
    double bstar = 0.0;                  // B* 大气阻尼系数（Bstar drag coefficient）
    double epoch_julian_date = constants::julian_date_j2000;  // 历元儒略日（Epoch Julian Date）
};

// 解析 TLE 数据行（Parse TLE data lines）
// line1: TLE 第一行; line2: TLE 第二行; tle: 输出解析结果
inline bool parse_tle(const std::string& line1, const std::string& line2, tle_record& tle) {
    if (line1.size() < 69 || line2.size() < 69) return false;
    auto parse_double = [](const std::string& s) { return std::atof(s.c_str()); };

    int epoch_year = std::atoi(line1.substr(18, 2).c_str());
    double epoch_day = parse_double(line1.substr(20, 12));
    epoch_year += (epoch_year < 57) ? 2000 : 1900;  // TLE 年份使用两位表示，57 表示 1957 年起点

    double jd_year_start = 1721424.5;
    int y = epoch_year - 1;
    jd_year_start += 365.0 * y + y / 4 - y / 100 + y / 400;
    tle.epoch_julian_date = jd_year_start + epoch_day;

    tle.inclination_rad = parse_double(line2.substr(8, 8)) * constants::deg_to_rad;
    tle.raan_rad = parse_double(line2.substr(17, 8)) * constants::deg_to_rad;
    tle.eccentricity = parse_double("0." + line2.substr(26, 7));
    tle.arg_perigee_rad = parse_double(line2.substr(34, 8)) * constants::deg_to_rad;
    tle.mean_anomaly_rad = parse_double(line2.substr(43, 8)) * constants::deg_to_rad;
    tle.mean_motion_rev_per_day = parse_double(line2.substr(52, 11));
    return true;
}

// 将 TLE 记录转换为轨道根数（Convert TLE record to orbital elements）
inline orbital_elements tle_to_orbital_elements(const tle_record& tle,
                                                double gravitational_parameter = mu_earth) {
    double n = tle.mean_motion_rev_per_day * constants::two_pi / 86400.0;  // 平均角速度（rad/s）
    double a = std::cbrt(gravitational_parameter / (n * n));               // 半长轴（m）
    orbital_elements oe;
    oe.semi_major_axis_m = a;
    oe.eccentricity = tle.eccentricity;
    oe.inclination_rad = tle.inclination_rad;
    oe.raan_rad = tle.raan_rad;
    oe.arg_periapsis_rad = tle.arg_perigee_rad;
    double E = solve_kepler(tle.mean_anomaly_rad, tle.eccentricity);
    oe.true_anomaly_rad = eccentric_to_true_anomaly(E, tle.eccentricity);
    return oe;
}

// 轻量版 SGP4 轨道推进（含 J2 长期项）（Lite SGP4 orbit propagation with J2 secular terms）
inline orbital_elements propagate_sgp4_lite(const tle_record& tle,
                                            double julian_date,
                                            const j2_params& params = {}) {
    orbital_elements oe = tle_to_orbital_elements(tle, params.gravitational_parameter);
    return propagate_j2_secular(oe, (julian_date - tle.epoch_julian_date) * 86400.0, params);
}

// 轻量版 SGP4 轨道推进，输出 ECI 位置和速度（Lite SGP4 propagation outputting ECI position and velocity）
inline void propagate_sgp4_lite(const tle_record& tle,
                                double julian_date,
                                vec3& position_eci,
                                vec3& velocity_eci,
                                const j2_params& params = {}) {
    orbital_elements oe = propagate_sgp4_lite(tle, julian_date, params);
    elements_to_state(oe, position_eci, velocity_eci);
}

}  // namespace xsf_math
