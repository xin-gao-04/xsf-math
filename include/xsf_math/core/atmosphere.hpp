#pragma once

#include "constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// International Standard Atmosphere (ISA) 1976 model
// Valid from -2 km to ~85 km geopotential altitude
struct atmosphere {

    // Temperature at altitude (K)
    // Supports troposphere (0-11km) and lower stratosphere (11-20km)
    static double temperature(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            return constants::ssl_temperature - constants::lapse_rate * alt_m;
        }
        // Lower stratosphere: isothermal
        if (alt_m <= 20000.0) {
            return constants::tropopause_temp;
        }
        // Upper stratosphere: temperature increases
        if (alt_m <= 32000.0) {
            return constants::tropopause_temp + 0.001 * (alt_m - 20000.0);
        }
        // Simplified above 32km
        return constants::tropopause_temp + 12.0 + 0.0028 * (alt_m - 32000.0);
    }

    // Temperature ratio (T / T_ssl)
    static double temperature_ratio(double alt_m) {
        return temperature(alt_m) / constants::ssl_temperature;
    }

    // Pressure at altitude (Pa)
    static double pressure(double alt_m) {
        if (alt_m <= constants::tropopause_alt) {
            double temp_ratio = temperature(alt_m) / constants::ssl_temperature;
            return constants::ssl_pressure * std::pow(temp_ratio, constants::gmr / constants::lapse_rate);
        }
        // Above tropopause: exponential decay
        double p_trop = pressure(constants::tropopause_alt);
        double dh = alt_m - constants::tropopause_alt;
        return p_trop * std::exp(-constants::gmr * dh / constants::tropopause_temp);
    }

    // Pressure ratio (P / P_ssl)
    static double pressure_ratio(double alt_m) {
        return pressure(alt_m) / constants::ssl_pressure;
    }

    // Air density at altitude (kg/m^3)
    static double density(double alt_m) {
        return pressure(alt_m) / (constants::gas_constant_air * temperature(alt_m));
    }

    // Density ratio (rho / rho_ssl)
    static double density_ratio(double alt_m) {
        return density(alt_m) / constants::ssl_density;
    }

    // Speed of sound at altitude (m/s)
    static double sonic_velocity(double alt_m) {
        return std::sqrt(constants::gamma_air * constants::gas_constant_air * temperature(alt_m));
    }

    // Dynamic pressure q = 0.5 * rho * V^2
    static double dynamic_pressure(double alt_m, double speed_mps) {
        return 0.5 * density(alt_m) * speed_mps * speed_mps;
    }

    // Mach number
    static double mach_number(double alt_m, double speed_mps) {
        return speed_mps / sonic_velocity(alt_m);
    }

    // True airspeed from Mach number
    static double speed_from_mach(double alt_m, double mach) {
        return mach * sonic_velocity(alt_m);
    }

    // Density altitude: altitude in standard atmosphere with same density
    static double density_altitude(double alt_m, double delta_temp_k = 0.0) {
        double T = temperature(alt_m) + delta_temp_k;
        double P = pressure(alt_m);
        double rho = P / (constants::gas_constant_air * T);
        // Invert density formula for troposphere
        // rho = rho_ssl * (T/T_ssl)^(g/(L*R) - 1)
        // Simplified Newton iteration for altitude given density
        double h = alt_m;
        for (int i = 0; i < 10; ++i) {
            double rho_h = density(h);
            double drho = (density(h + 1.0) - density(h - 1.0)) / 2.0;
            if (std::abs(drho) < 1e-20) break;
            h -= (rho_h - rho) / drho;
        }
        return h;
    }

    // Dynamic viscosity (Sutherland's law) in kg/(m*s)
    static double dynamic_viscosity(double alt_m) {
        double T = temperature(alt_m);
        // Sutherland's formula: mu = mu_ref * (T/T_ref)^(3/2) * (T_ref+S)/(T+S)
        constexpr double mu_ref = 1.716e-5;  // reference viscosity at T_ref
        constexpr double T_ref  = 273.15;     // reference temperature (K)
        constexpr double S      = 110.4;      // Sutherland constant (K)
        return mu_ref * std::pow(T / T_ref, 1.5) * (T_ref + S) / (T + S);
    }

    // Kinematic viscosity in m^2/s
    static double kinematic_viscosity(double alt_m) {
        return dynamic_viscosity(alt_m) / density(alt_m);
    }

    // Non-standard atmosphere: apply temperature offset
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
