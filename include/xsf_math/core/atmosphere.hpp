#pragma once

#include "constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// 1976 年国际标准大气（ISA）模型
// 适用范围为 -2 km 到约 85 km 位势高度
struct atmosphere {

    // 指定高度的温度（K）
    // 支持对流层（0-11km）和下平流层（11-20km）
    static double temperature(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            return constants::ssl_temperature - constants::lapse_rate * alt_m;
        }
        // 下平流层：等温
        if (alt_m <= 20000.0) {
            return constants::tropopause_temp;
        }
        // 上平流层：温度上升
        if (alt_m <= 32000.0) {
            return constants::tropopause_temp + 0.001 * (alt_m - 20000.0);
        }
        // 32km 以上的简化模型
        return constants::tropopause_temp + 12.0 + 0.0028 * (alt_m - 32000.0);
    }

    // 温度比（T / T_ssl）
    static double temperature_ratio(double alt_m) {
        return temperature(alt_m) / constants::ssl_temperature;
    }

    // 指定高度的压力（Pa）
    static double pressure(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            double temp_ratio = temperature(alt_m) / constants::ssl_temperature;
            return constants::ssl_pressure * std::pow(temp_ratio, constants::gmr / constants::lapse_rate);
        }
        // 对流层顶以上：指数衰减
        double p_trop = pressure(constants::tropopause_alt);
        double dh = alt_m - constants::tropopause_alt;
        return p_trop * std::exp(-constants::gmr * dh / constants::tropopause_temp);
    }

    // 压力比（P / P_ssl）
    static double pressure_ratio(double alt_m) {
        return pressure(alt_m) / constants::ssl_pressure;
    }

    // 指定高度的空气密度（kg/m^3）
    static double density(double alt_m) {
        return pressure(alt_m) / (constants::gas_constant_air * temperature(alt_m));
    }

    // 密度比（rho / rho_ssl）
    static double density_ratio(double alt_m) {
        return density(alt_m) / constants::ssl_density;
    }

    // 指定高度的声速（m/s）
    static double sonic_velocity(double alt_m) {
        return std::sqrt(constants::gamma_air * constants::gas_constant_air * temperature(alt_m));
    }

    // 动压 q = 0.5 * rho * V^2
    static double dynamic_pressure(double alt_m, double speed_mps) {
        return 0.5 * density(alt_m) * speed_mps * speed_mps;
    }

    // 马赫数
    static double mach_number(double alt_m, double speed_mps) {
        return speed_mps / sonic_velocity(alt_m);
    }

    // 根据马赫数反算真空速
    static double speed_from_mach(double alt_m, double mach) {
        return mach * sonic_velocity(alt_m);
    }

    // 密度高度：标准大气中与当前密度相同的高度
    static double density_altitude(double alt_m, double delta_temp_k = 0.0) {
        double T = temperature(alt_m) + delta_temp_k;
        double P = pressure(alt_m);
        double rho = P / (constants::gas_constant_air * T);
        // 对流层密度公式求逆
        // rho = rho_ssl * (T/T_ssl)^(g/(L*R) - 1)
        // 使用简化的牛顿迭代求解密度对应的高度
        double h = alt_m;
        for (int i = 0; i < 10; ++i) {
            double rho_h = density(h);
            double drho = (density(h + 1.0) - density(h - 1.0)) / 2.0;
            if (std::abs(drho) < 1e-20) break;
            h -= (rho_h - rho) / drho;
        }
        return h;
    }

    // 动力黏度（Sutherland 定律），单位 kg/(m*s)
    static double dynamic_viscosity(double alt_m) {
        double T = temperature(alt_m);
        // Sutherland 公式：mu = mu_ref * (T/T_ref)^(3/2) * (T_ref+S)/(T+S)
        constexpr double mu_ref = 1.716e-5;  // T_ref 下的参考黏度
        constexpr double T_ref  = 273.15;     // 参考温度（K）
        constexpr double S      = 110.4;      // Sutherland 常数（K）
        return mu_ref * std::pow(T / T_ref, 1.5) * (T_ref + S) / (T + S);
    }

    // 运动黏度，单位 m^2/s
    static double kinematic_viscosity(double alt_m) {
        return dynamic_viscosity(alt_m) / density(alt_m);
    }

    // 非标准大气：施加温度偏差
    struct non_standard {
        double delta_temp_k = 0.0;

        double temperature(double alt_m) const {
            return atmosphere::temperature(alt_m) + delta_temp_k;
        }

        double density(double alt_m) const {
            return atmosphere::pressure(alt_m) / (constants::gas_constant_air * temperature(alt_m));
        }

        double sonic_velocity(double alt_m) const {
            return std::sqrt(constants::gamma_air * constants::gas_constant_air * temperature(alt_m));
        }

        double dynamic_pressure(double alt_m, double speed_mps) const {
            return 0.5 * density(alt_m) * speed_mps * speed_mps;
        }

        double mach_number(double alt_m, double speed_mps) const {
            return speed_mps / sonic_velocity(alt_m);
        }
    };
};

} // namespace xsf_math
