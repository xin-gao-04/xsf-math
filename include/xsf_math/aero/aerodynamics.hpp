#pragma once

#include "../core/constants.hpp"
#include "../core/atmosphere.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace xsf_math {

// 气动力计算

// 基础气动系数
struct aero_coefficients {
    double cl = 0.0;     // 升力系数
    double cd = 0.0;     // 阻力系数
    double cy = 0.0;     // 侧力系数
};

// 基础气动状态
struct aero_state {
    double altitude_m     = 0.0;
    double speed_mps      = 0.0;
    double mach           = 0.0;
    double dynamic_pressure_pa = 0.0;
    double air_density    = 0.0;
    double angle_of_attack_rad = 0.0;  // 迎角 alpha
    double sideslip_angle_rad  = 0.0;  // 侧滑角 beta

    // 根据高度和速度计算
    static aero_state from_alt_speed(double alt_m, double speed_mps) {
        aero_state s;
        s.altitude_m = alt_m;
        s.speed_mps = speed_mps;
        s.air_density = atmosphere::density(alt_m);
        s.dynamic_pressure_pa = 0.5 * s.air_density * speed_mps * speed_mps;
        s.mach = atmosphere::mach_number(alt_m, speed_mps);
        return s;
    }
};

// 机体系（ECS）中的气动力
struct aero_forces {
    double drag_n   = 0.0;  // 沿 -X 方向（与前进方向相反）
    double side_n   = 0.0;  // 沿 Y 方向（右机翼）
    double normal_n = 0.0;  // 沿 Z 方向（向下）
    double lift_n   = 0.0;  // 垂直于速度方向（派生量）

    // 合力大小
    double total() const {
        return std::sqrt(drag_n*drag_n + side_n*side_n + normal_n*normal_n);
    }
};

// 二维气动模型（升力 + 阻力，抛物线阻力极线）
// Cd = Cd0 + CL^2 / (pi * AR * e)
struct aero_2d {
    double ref_area_m2   = 1.0;    // 参考面积（翼面积）
    double aspect_ratio  = 6.0;    // AR = 展弦比 = 翼展^2 / 面积
    double oswald_factor = 0.8;    // e（奥斯瓦尔德效率）
    double cl_max        = 1.5;    // 失速前最大 CL
    double lift_curve_slope = 0.1; // dCL/dalpha（每度），约等于 2*pi/rad

    // 零升阻力系数（可随马赫数变化）
    double cd0 = 0.02;

    // 阻力系数表：Cd0 与马赫数的对应关系（可选）
    std::vector<double> cd0_mach_table;   // 马赫数分段点
    std::vector<double> cd0_values;       // 每个马赫数点对应的 Cd0

    // 诱导阻力系数 K = 1 / (pi * AR * e)
    double induced_drag_factor() const {
        return 1.0 / (constants::pi * aspect_ratio * oswald_factor);
    }

    // 根据给定马赫数获取 Cd0（查表或常量）
    double get_cd0(double mach) const {
        if (cd0_mach_table.empty() || cd0_values.empty()) return cd0;
        return table_lookup(cd0_mach_table, cd0_values, mach);
    }

    // 计算气动力
    aero_forces compute(const aero_state& state, double requested_g_normal = 0.0,
                         double requested_g_lateral = 0.0) const {
        double q = state.dynamic_pressure_pa;
        double S = ref_area_m2;
        double qS = q * S;

        if (qS < 1e-10) return {};

        // 根据请求的法向力求 CL（或根据迎角求得）
        double cl;
        if (std::abs(requested_g_normal) > 1e-10) {
            // 计算满足请求气动力所需的 CL
            double weight_equivalent = requested_g_normal * constants::gravity_mps2;
            cl = weight_equivalent / qS;
        } else {
            cl = lift_curve_slope * (state.angle_of_attack_rad * constants::rad_to_deg);
        }

        // 将 CL 限制到最大值
        cl = clamp(cl, -cl_max, cl_max);

        // 阻力极线
        double cd0_m = get_cd0(state.mach);
        double K = induced_drag_factor();
        double cd_total = cd0_m + K * cl * cl;

        aero_forces f;
        f.drag_n   = qS * cd_total;
        f.lift_n   = qS * cl;
        f.normal_n = -f.lift_n;  // normal force in body Z (down)

        // 侧向力
        if (std::abs(requested_g_lateral) > 1e-10) {
            double cy = requested_g_lateral * constants::gravity_mps2 / qS;
            f.side_n = qS * cy;
        }

        return f;
    }

    // 最大可用过载
    double max_g_available(const aero_state& state, double mass_kg) const {
        if (mass_kg <= 0.0) return 0.0;
        double qS = state.dynamic_pressure_pa * ref_area_m2;
        double max_force = qS * cl_max;
        return max_force / (mass_kg * constants::gravity_mps2);
    }

    // 比剩余功率（Ps）
    double specific_excess_power(const aero_state& state, double thrust_n,
                                  double mass_kg, double cl_current) const {
        if (mass_kg <= 0.0) return 0.0;
        double qS = state.dynamic_pressure_pa * ref_area_m2;
        double cd0_m = get_cd0(state.mach);
        double K = induced_drag_factor();
        double drag = qS * (cd0_m + K * cl_current * cl_current);
        return (thrust_n - drag) * state.speed_mps / (mass_kg * constants::gravity_mps2);
    }
};

// 燃油消耗模型
struct fuel_model {
    double initial_fuel_kg = 100.0;
    double current_fuel_kg = 100.0;

    // 比油耗：kg/(N*s)
    double sfc = 0.0001;  // 推力相关

    // 按给定推力和时间消耗燃油
    double consume(double thrust_n, double dt_s) {
        double fuel_used = sfc * thrust_n * dt_s;
        fuel_used = std::min(fuel_used, current_fuel_kg);
        current_fuel_kg -= fuel_used;
        return fuel_used;
    }

    bool is_empty() const { return current_fuel_kg <= 0.0; }
    double fuel_fraction() const {
        return (initial_fuel_kg > 0.0) ? current_fuel_kg / initial_fuel_kg : 0.0;
    }
};

// 简单火箭发动机模型
struct rocket_motor {
    double thrust_n     = 10000.0;   // 标称推力
    double burn_time_s  = 10.0;      // 总燃烧时间
    double isp_s        = 250.0;     // 比冲
    double elapsed_s    = 0.0;

    bool is_burning() const { return elapsed_s < burn_time_s; }

    double current_thrust(double dt_s) {
        if (!is_burning()) return 0.0;
        elapsed_s += dt_s;
        if (elapsed_s >= burn_time_s) {
            return thrust_n * (burn_time_s - (elapsed_s - dt_s)) / dt_s;
        }
        return thrust_n;
    }

    // 燃料消耗率（kg/s）
    double fuel_rate() const {
        if (!is_burning()) return 0.0;
        return thrust_n / (isp_s * constants::gravity_mps2);
    }
};

// 失速检测
inline bool is_stalled(double aoa_rad, double critical_aoa_rad = 15.0 * constants::deg_to_rad) {
    return std::abs(aoa_rad) > critical_aoa_rad;
}

// 飞行包线检查
struct flight_envelope {
    double max_mach      = 3.0;
    double max_altitude_m = 25000.0;
    double max_g          = 9.0;
    double min_speed_mps  = 50.0;

    bool within_envelope(double mach, double alt_m, double g_load, double speed_mps) const {
        return mach <= max_mach &&
               alt_m <= max_altitude_m &&
               std::abs(g_load) <= max_g &&
               speed_mps >= min_speed_mps;
    }
};

} // namespace xsf_math
