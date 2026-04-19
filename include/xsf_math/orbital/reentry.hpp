#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

// 再入飞行器状态（Reentry vehicle state）
struct reentry_state {
    double altitude_m = 120000.0;                // 海拔高度（Altitude），单位 m
    double downrange_m = 0.0;                    // 地面射程（Downrange distance），单位 m
    double velocity_mps = 7500.0;                // 速度（Velocity），单位 m/s
    double flight_path_angle_rad = -5.0 * constants::deg_to_rad;  // 航迹角（Flight path angle），单位 rad；负值表示下降
    double mass_kg = 1000.0;                     // 质量（Mass），单位 kg
    double heat_load_j_cm2 = 0.0;                // 累积热载荷（Cumulative heat load），单位 J/cm^2
};

// 再入飞行器气动参数（Reentry vehicle aerodynamic parameters）
struct reentry_vehicle {
    double reference_area_m2 = 1.0;    // 参考面积（Reference area），单位 m^2
    double drag_coefficient = 1.2;     // 阻力系数（Drag coefficient, Cd）
    double lift_to_drag = 0.0;         // 升阻比（Lift-to-drag ratio, L/D）
    double ballistic_coeff_kg_m2 = 300.0;  // 弹道系数（Ballistic coefficient），单位 kg/m^2
};

// 再入环境参数（Reentry environment parameters）
struct reentry_environment {
    double sea_level_density_kg_m3 = constants::ssl_density;  // 海平面大气密度（Sea-level density），单位 kg/m^3
    double scale_height_m = 7200.0;    // 大气标度高度（Scale height），单位 m
    double nose_radius_m = 0.5;        // 头部半径（Nose radius），单位 m
};

// 再入单步计算结果（Reentry single-step result）
struct reentry_step_result {
    double density_kg_m3 = 0.0;           // 当地大气密度（Atmospheric density），单位 kg/m^3
    double dynamic_pressure_pa = 0.0;     // 动压（Dynamic pressure），单位 Pa
    double convective_heat_flux_w_m2 = 0.0; // 对流热流密度（Convective heat flux），单位 W/m^2
    double deceleration_mps2 = 0.0;       // 减速度（Deceleration），单位 m/s^2
};

// 再入轨道推进器（Reentry trajectory propagator）
struct reentry_propagator {
    reentry_vehicle vehicle{};        // 再入飞行器参数（Reentry vehicle parameters）
    reentry_environment environment{};// 环境参数（Environment parameters）

    // 计算给定高度的大气密度（Calculate atmospheric density at given altitude）
    double atmosphere_density(double altitude_m) const {
        return environment.sea_level_density_kg_m3 *
               std::exp(-std::max(altitude_m, 0.0) / std::max(environment.scale_height_m, 1.0));
    }

    // 单步推进再入状态（Single-step reentry propagation）
    // state: 输入/输出状态; dt_s: 时间步长（s）
    reentry_step_result step(reentry_state& state, double dt_s) const {
        reentry_step_result out;
        if (dt_s <= 0.0) return out;

        double radius = constants::earth_radius_m + std::max(state.altitude_m, 0.0);  // 地心距（m）
        double g = constants::gravity_mps2 * (constants::earth_radius_m * constants::earth_radius_m) / (radius * radius);  // 当地重力加速度（m/s^2）
        out.density_kg_m3 = atmosphere_density(state.altitude_m);
        out.dynamic_pressure_pa = 0.5 * out.density_kg_m3 * state.velocity_mps * state.velocity_mps;

        double drag = out.dynamic_pressure_pa * vehicle.reference_area_m2 * vehicle.drag_coefficient;  // 阻力（N）
        double lift = drag * vehicle.lift_to_drag;  // 升力（N）
        double vdot = -(drag / std::max(state.mass_kg, 1.0)) - g * std::sin(state.flight_path_angle_rad);  // 速度变化率（m/s^2）
        double gamma_dot = lift / std::max(state.mass_kg * std::max(state.velocity_mps, 1.0), 1.0) +
                           (state.velocity_mps / radius - g / std::max(state.velocity_mps, 1.0)) *
                               std::cos(state.flight_path_angle_rad);  // 航迹角变化率（rad/s）
        double hdot = state.velocity_mps * std::sin(state.flight_path_angle_rad);  // 高度变化率（m/s）
        double xdot = state.velocity_mps * std::cos(state.flight_path_angle_rad);  // 射程变化率（m/s）

        state.velocity_mps = std::max(state.velocity_mps + vdot * dt_s, 0.0);
        state.flight_path_angle_rad += gamma_dot * dt_s;
        state.altitude_m = std::max(state.altitude_m + hdot * dt_s, 0.0);
        state.downrange_m += xdot * dt_s;

        // Allen-Eggers 简化对流热流公式（Simplified convective heating rate）
        out.convective_heat_flux_w_m2 =
            1.83e-4 * std::sqrt(out.density_kg_m3 / std::max(environment.nose_radius_m, 1.0e-3)) *
            std::pow(state.velocity_mps, 3.0);
        state.heat_load_j_cm2 += out.convective_heat_flux_w_m2 * dt_s / 1.0e4;  // W/m^2 * s / 1e4 = J/cm^2
        out.deceleration_mps2 = -vdot;
        return out;
    }
};

}  // namespace xsf_math
