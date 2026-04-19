#pragma once

#include "constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

/** 1976 年国际标准大气（ISA）模型，适用范围为 -2 km 到约 85 km 位势高度 (1976 International Standard Atmosphere (ISA) model, valid from -2 km to approx 85 km geopotential altitude) */
struct atmosphere {

    /** 指定高度的温度（K） (Temperature at given altitude, in K) */
    static double temperature(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            return constants::ssl_temperature - constants::lapse_rate * alt_m;
        }
        // 下平流层：等温 (Lower stratosphere: isothermal)
        if (alt_m <= 20000.0) {
            return constants::tropopause_temp;
        }
        // 上平流层：温度上升 (Upper stratosphere: temperature increases)
        if (alt_m <= 32000.0) {
            return constants::tropopause_temp + 0.001 * (alt_m - 20000.0);
        }
        // 32km 以上的简化模型 (Simplified model above 32 km)
        return constants::tropopause_temp + 12.0 + 0.0028 * (alt_m - 32000.0);
    }

    /** 温度比（T / T_ssl） (Temperature ratio) */
    static double temperature_ratio(double alt_m) {
        return temperature(alt_m) / constants::ssl_temperature;
    }

    /** 指定高度的压力（Pa） (Pressure at given altitude, in Pa) */
    static double pressure(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            double temp_ratio = temperature(alt_m) / constants::ssl_temperature;
            return constants::ssl_pressure * std::pow(temp_ratio, constants::gmr / constants::lapse_rate);
        }
        // 对流层顶以上：指数衰减 (Above tropopause: exponential decay)
        double p_trop = pressure(constants::tropopause_alt);
        double dh = alt_m - constants::tropopause_alt;
        return p_trop * std::exp(-constants::gmr * dh / constants::tropopause_temp);
    }

    /** 压力比（P / P_ssl） (Pressure ratio) */
    static double pressure_ratio(double alt_m) {
        return pressure(alt_m) / constants::ssl_pressure;
    }

    /** 指定高度的空气密度（kg/m³） (Air density at given altitude, in kg/m³) */
    static double density(double alt_m) {
        return pressure(alt_m) / (constants::gas_constant_air * temperature(alt_m));
    }

    /** 密度比（rho / rho_ssl） (Density ratio) */
    static double density_ratio(double alt_m) {
        return density(alt_m) / constants::ssl_density;
    }

    /** 指定高度的声速（m/s） (Speed of sound at given altitude, in m/s) */
    static double sonic_velocity(double alt_m) {
        return std::sqrt(constants::gamma_air * constants::gas_constant_air * temperature(alt_m));
    }

    /** 动压 q = 0.5 * rho * V² (Dynamic pressure q = 0.5 * rho * V²) */
    static double dynamic_pressure(double alt_m, double speed_mps) {
        return 0.5 * density(alt_m) * speed_mps * speed_mps;
    }

    /** 马赫数 (Mach number) */
    static double mach_number(double alt_m, double speed_mps) {
        return speed_mps / sonic_velocity(alt_m);
    }

    /** 根据马赫数反算真空速（True Airspeed from Mach number） (Compute true airspeed from Mach number) */
    static double speed_from_mach(double alt_m, double mach) {
        return mach * sonic_velocity(alt_m);
    }

    /** 密度高度：标准大气中与当前密度相同的高度 (Density altitude: altitude in standard atmosphere with the same density) */
    static double density_altitude(double alt_m, double delta_temp_k = 0.0) {
        double T = temperature(alt_m) + delta_temp_k;
        double P = pressure(alt_m);
        double rho = P / (constants::gas_constant_air * T);
        // 对流层密度公式求逆 (Invert troposphere density formula)
        // rho = rho_ssl * (T/T_ssl)^(g/(L*R) - 1)
        // 使用简化的牛顿迭代求解密度对应的高度 (Use simplified Newton iteration to solve for altitude corresponding to density)
        double h = alt_m;
        for (int i = 0; i < 10; ++i) {
            double rho_h = density(h);
            double drho = (density(h + 1.0) - density(h - 1.0)) / 2.0;
            if (std::abs(drho) < 1e-20) break;
            h -= (rho_h - rho) / drho;
        }
        return h;
    }

    /** 动力黏度（Sutherland 定律），单位 kg/(m·s) (Dynamic viscosity via Sutherland's law, in kg/(m·s)) */
    static double dynamic_viscosity(double alt_m) {
        double T = temperature(alt_m);
        // Sutherland 公式 (Sutherland formula)：mu = mu_ref * (T/T_ref)^(3/2) * (T_ref+S)/(T+S)
        constexpr double mu_ref = 1.716e-5;  ///< 参考黏度 (reference viscosity)，T_ref 下
        constexpr double T_ref  = 273.15;     ///< 参考温度 (reference temperature)，单位 K
        constexpr double S      = 110.4;      ///< Sutherland 常数 (Sutherland constant)，单位 K
        return mu_ref * std::pow(T / T_ref, 1.5) * (T_ref + S) / (T + S);
    }

    /** 运动黏度，单位 m²/s (Kinematic viscosity, in m²/s) */
    static double kinematic_viscosity(double alt_m) {
        return dynamic_viscosity(alt_m) / density(alt_m);
    }

    /** 非标准大气：施加温度偏差 (Non-standard atmosphere: apply temperature offset) */
    struct non_standard {
        double delta_temp_k = 0.0; ///< 温度偏差 (temperature offset)，单位 K

        /** 非标准大气温度 (Non-standard atmosphere temperature) */
        double temperature(double alt_m) const {
            return atmosphere::temperature(alt_m) + delta_temp_k;
        }

        /** 非标准大气密度 (Non-standard atmosphere density) */
        double density(double alt_m) const {
            return atmosphere::pressure(alt_m) / (constants::gas_constant_air * temperature(alt_m));
        }

        /** 非标准大气声速 (Non-standard atmosphere speed of sound) */
        double sonic_velocity(double alt_m) const {
            return std::sqrt(constants::gamma_air * constants::gas_constant_air * temperature(alt_m));
        }

        /** 非标准大气动压 (Non-standard atmosphere dynamic pressure) */
        double dynamic_pressure(double alt_m, double speed_mps) const {
            return 0.5 * density(alt_m) * speed_mps * speed_mps;
        }

        /** 非标准大气马赫数 (Non-standard atmosphere Mach number) */
        double mach_number(double alt_m, double speed_mps) const {
            return speed_mps / sonic_velocity(alt_m);
        }
    };
};

} // namespace xsf_math
