#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 开普勒轨道力学

// 地球引力参数（mu）
constexpr double mu_earth = 3.986004418e14;  // m^3/s^2

// 经典轨道根数
struct orbital_elements {
    double semi_major_axis_m;  // a
    double eccentricity;       // e（0=圆轨道，0<e<1=椭圆，1=抛物线，>1=双曲线）
    double inclination_rad;    // i
    double raan_rad;           // Omega（升交点赤经）
    double arg_periapsis_rad;  // omega（近地点幅角）
    double true_anomaly_rad;   // nu（真近点角）

    // 轨道周期（秒）
    double period() const {
        if (eccentricity >= 1.0 || semi_major_axis_m <= 0.0) return 1e20;
        return 2.0 * constants::pi * std::sqrt(
            std::pow(semi_major_axis_m, 3) / mu_earth);
    }

    // 平均角速度（rad/s）
    double mean_motion() const {
        if (semi_major_axis_m <= 0.0) return 0.0;
        return std::sqrt(mu_earth / std::pow(semi_major_axis_m, 3));
    }

    // 近地点和远地点半径（地心距离）
    double periapsis_radius() const {
        return semi_major_axis_m * (1.0 - eccentricity);
    }
    double apoapsis_radius() const {
        if (eccentricity >= 1.0) return 1e20;
        return semi_major_axis_m * (1.0 + eccentricity);
    }

    // 相对地表的高度
    double periapsis_altitude() const { return periapsis_radius() - constants::earth_radius_m; }
    double apoapsis_altitude() const { return apoapsis_radius() - constants::earth_radius_m; }

    // 当前轨道位置的速度
    double velocity(double radius_m) const {
        // 视-viva 方程：v^2 = mu * (2/r - 1/a)
        return std::sqrt(mu_earth * (2.0/radius_m - 1.0/semi_major_axis_m));
    }

    // 当前真近点角对应的半径
    double radius() const {
        double p = semi_major_axis_m * (1.0 - eccentricity * eccentricity);
        return p / (1.0 + eccentricity * std::cos(true_anomaly_rad));
    }
};

// 求解开普勒方程：M = E - e*sin(E)
// 已知平近点角 M 和偏心率 e，求偏近点角 E
inline double solve_kepler(double M_rad, double eccentricity, int max_iter = 30) {
    // 初始猜测
    double E = M_rad;
    if (eccentricity > 0.8) {
        E = constants::pi;
    }

    // 牛顿-拉夫森迭代
    for (int i = 0; i < max_iter; ++i) {
        double f  = E - eccentricity * std::sin(E) - M_rad;
        double fp = 1.0 - eccentricity * std::cos(E);
        if (std::abs(fp) < 1e-20) break;
        double dE = f / fp;
        E -= dE;
        if (std::abs(dE) < 1e-12) break;
    }
    return E;
}

// 将偏近点角转换为真近点角
inline double eccentric_to_true_anomaly(double E_rad, double eccentricity) {
    double half_E = E_rad / 2.0;
    double factor = std::sqrt((1.0 + eccentricity) / (1.0 - eccentricity));
    return 2.0 * std::atan2(factor * std::sin(half_E), std::cos(half_E));
}

// 将真近点角转换为偏近点角
inline double true_to_eccentric_anomaly(double nu_rad, double eccentricity) {
    return 2.0 * std::atan2(
        std::sqrt(1.0 - eccentricity) * std::sin(nu_rad / 2.0),
        std::sqrt(1.0 + eccentricity) * std::cos(nu_rad / 2.0));
}

// 由偏近点角求平近点角
inline double eccentric_to_mean_anomaly(double E_rad, double eccentricity) {
    return E_rad - eccentricity * std::sin(E_rad);
}

// 按时间增量推进轨道（二体开普勒模型）
inline orbital_elements propagate_kepler(const orbital_elements& oe, double dt_s) {
    double n = oe.mean_motion();

    // 当前平近点角
    double E0 = true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity);
    double M0 = eccentric_to_mean_anomaly(E0, oe.eccentricity);

    // 新的平近点角
    double M = std::fmod(M0 + n * dt_s, constants::two_pi);
    if (M < 0) M += constants::two_pi;

    // 求解新的偏近点角
    double E = solve_kepler(M, oe.eccentricity);

    // 新的真近点角
    double nu = eccentric_to_true_anomaly(E, oe.eccentricity);

    orbital_elements result = oe;
    result.true_anomaly_rad = nu;
    return result;
}

// 将轨道根数转换为 ECI 位置和速度
inline void elements_to_state(const orbital_elements& oe, vec3& pos, vec3& vel) {
    double e = oe.eccentricity;
    double a = oe.semi_major_axis_m;
    double nu = oe.true_anomaly_rad;

    double p = a * (1.0 - e*e);
    double r = p / (1.0 + e * std::cos(nu));

    // 轨道平面中的位置
    double x_orb = r * std::cos(nu);
    double y_orb = r * std::sin(nu);

    // 轨道平面中的速度
    double h = std::sqrt(mu_earth * p);
    double vx_orb = -(mu_earth / h) * std::sin(nu);
    double vy_orb =  (mu_earth / h) * (e + std::cos(nu));

    // 从轨道平面旋转到 ECI
    double ci = std::cos(oe.inclination_rad);
    double si = std::sin(oe.inclination_rad);
    double cO = std::cos(oe.raan_rad);
    double sO = std::sin(oe.raan_rad);
    double cw = std::cos(oe.arg_periapsis_rad);
    double sw = std::sin(oe.arg_periapsis_rad);

    // 旋转矩阵元素
    double r11 = cO*cw - sO*sw*ci;
    double r12 = -(cO*sw + sO*cw*ci);
    double r21 = sO*cw + cO*sw*ci;
    double r22 = -(sO*sw - cO*cw*ci);
    double r31 = sw*si;
    double r32 = cw*si;

    pos = { r11*x_orb + r12*y_orb,
            r21*x_orb + r22*y_orb,
            r31*x_orb + r32*y_orb };

    vel = { r11*vx_orb + r12*vy_orb,
            r21*vx_orb + r22*vy_orb,
            r31*vx_orb + r32*vy_orb };
}

// 两颗卫星之间的视线可见性
// 如果连线不与地球相交则返回 true
inline bool los_visible(const vec3& pos_a, const vec3& pos_b) {
    vec3 d = pos_b - pos_a;
    double a = d.magnitude_sq();
    double b = 2.0 * pos_a.dot(d);
    double c = pos_a.magnitude_sq() - constants::earth_radius_m * constants::earth_radius_m;

    double discriminant = b*b - 4.0*a*c;
    if (discriminant < 0.0) return true;  // 无交点

    double sqrt_disc = std::sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2.0*a);
    double t2 = (-b + sqrt_disc) / (2.0*a);

    // 在区间 [0,1] 内有交点则表示被遮挡
    return !(t1 >= 0.0 && t1 <= 1.0) && !(t2 >= 0.0 && t2 <= 1.0);
}

// 大气阻力减速度（简化）
inline double atmospheric_drag_accel(double altitude_m, double speed_mps,
                                      double drag_coeff_cd, double area_m2,
                                      double mass_kg) {
    // 用于轨道高度的指数大气密度模型
    constexpr double rho0 = 1.225;
    constexpr double H = 8500.0;  // 标度高度（m）
    double rho = rho0 * std::exp(-altitude_m / H);

    double drag_force = 0.5 * rho * speed_mps * speed_mps * drag_coeff_cd * area_m2;
    return (mass_kg > 0.0) ? drag_force / mass_kg : 0.0;
}

} // namespace xsf_math
