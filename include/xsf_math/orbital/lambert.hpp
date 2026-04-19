#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include "kepler.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 兰伯特问题求解结果（Lambert problem solution）
struct lambert_solution {
    bool valid = false;                 // 解是否有效（Solution valid flag）
    vec3 departure_velocity_eci{};      // 出发时刻 ECI 速度（Departure velocity in ECI），单位 m/s
    vec3 arrival_velocity_eci{};        // 到达时刻 ECI 速度（Arrival velocity in ECI），单位 m/s
    double semi_major_axis_m = 0.0;     // 转移轨道半长轴（Transfer semi-major axis），单位 m
    double transfer_angle_rad = 0.0;    // 转移角（Transfer angle），单位 rad
    double tof_s = 0.0;                 // 飞行时间（Time of flight），单位 s
    int iterations = 0;                 // 迭代次数（Iteration count）
};

namespace detail {

// Stumpff 函数 c(z)（Stumpff function c of z）
inline double stumpff_c(double z) {
    if (z > 1.0e-8) return (1.0 - std::cos(std::sqrt(z))) / z;
    if (z < -1.0e-8) return (std::cosh(std::sqrt(-z)) - 1.0) / (-z);
    return 0.5;
}

// Stumpff 函数 s(z)（Stumpff function s of z）
inline double stumpff_s(double z) {
    if (z > 1.0e-8) {
        double rz = std::sqrt(z);
        return (rz - std::sin(rz)) / (rz * rz * rz);
    }
    if (z < -1.0e-8) {
        double rz = std::sqrt(-z);
        return (std::sinh(rz) - rz) / (rz * rz * rz);
    }
    return 1.0 / 6.0;
}

// 由 z 参数计算飞行时间（Compute time of flight from z parameter）
// A: 三角形的面积相关参数（m^1.5）; r1, r2: 初始和终了半径（m）; mu: 引力参数（m^3/s^2）
inline double time_of_flight_from_z(double z, double A, double r1, double r2, double mu) {
    double C = stumpff_c(z);
    double S = stumpff_s(z);
    if (C <= 1.0e-12) return 1.0e30;
    double y = r1 + r2 + A * (z * S - 1.0) / std::sqrt(C);
    if (y <= 0.0) return 1.0e30;
    double x = std::sqrt(y / C);
    return (x * x * x * S + A * std::sqrt(y)) / std::sqrt(mu);
}

}  // namespace detail

// 求解兰伯特问题（Solve Lambert's problem）
// r1_eci: 初始位置（ECI，m）; r2_eci: 终了位置（ECI，m）; tof_s: 飞行时间（s）
// long_way: 是否走长弧（long-way transfer）; gravitational_parameter: 引力参数（m^3/s^2，默认地球）
inline lambert_solution solve_lambert(const vec3& r1_eci,
                                      const vec3& r2_eci,
                                      double tof_s,
                                      bool long_way = false,
                                      double gravitational_parameter = mu_earth,
                                      int max_iter = 50) {
    lambert_solution out;
    out.tof_s = tof_s;
    double r1 = r1_eci.magnitude();
    double r2 = r2_eci.magnitude();
    if (r1 <= 0.0 || r2 <= 0.0 || tof_s <= 0.0) return out;

    double cos_dnu = std::clamp(r1_eci.dot(r2_eci) / (r1 * r2), -1.0, 1.0);
    double sin_dnu = r1_eci.cross(r2_eci).magnitude() / (r1 * r2);
    if (long_way) sin_dnu = -sin_dnu;  // 长弧取反（Reverse for long-way transfer）
    double dnu = std::atan2(sin_dnu, cos_dnu);
    if (dnu < 0.0) dnu += constants::two_pi;
    out.transfer_angle_rad = dnu;

    double A = std::sin(dnu) * std::sqrt(r1 * r2 / std::max(1.0 - std::cos(dnu), 1.0e-12));
    if (std::abs(A) < 1.0e-12) return out;

    double z_low = -4.0 * constants::pi * constants::pi;
    double z_high = 4.0 * constants::pi * constants::pi;
    double z = 0.0;
    for (int i = 0; i < max_iter; ++i) {
        z = 0.5 * (z_low + z_high);
        double tof_mid = detail::time_of_flight_from_z(z, A, r1, r2, gravitational_parameter);
        out.iterations = i + 1;
        if (std::abs(tof_mid - tof_s) < 1.0e-6) break;
        if (tof_mid > tof_s) z_high = z;
        else z_low = z;
    }

    double C = detail::stumpff_c(z);
    double S = detail::stumpff_s(z);
    double y = r1 + r2 + A * (z * S - 1.0) / std::sqrt(std::max(C, 1.0e-12));
    if (y <= 0.0) return out;

    double f = 1.0 - y / r1;
    double g = A * std::sqrt(y / gravitational_parameter);
    double gdot = 1.0 - y / r2;
    if (std::abs(g) < 1.0e-12) return out;

    out.departure_velocity_eci = (r2_eci - r1_eci * f) / g;
    out.arrival_velocity_eci = (r2_eci * gdot - r1_eci) / g;
    out.semi_major_axis_m = 0.5 * (r1 + r2 + y);
    out.valid = true;
    return out;
}

}  // namespace xsf_math
