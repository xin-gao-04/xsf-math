#pragma once

#include <cmath>
#include <limits>

namespace xsf_math {

struct constants {
    static constexpr double pi            = 3.14159265358979323846;
    static constexpr double two_pi        = 2.0 * pi;
    static constexpr double half_pi       = pi / 2.0;
    static constexpr double deg_to_rad    = pi / 180.0;
    static constexpr double rad_to_deg    = 180.0 / pi;

    // Physics
    static constexpr double speed_of_light  = 299792458.0;        // m/s
    static constexpr double boltzmann_k     = 1.380649e-23;       // J/K
    static constexpr double earth_radius_m  = 6371000.0;          // m (mean)
    static constexpr double gravity_mps2    = 9.80665;            // m/s^2 (standard)

    // Atmosphere (ISA sea level)
    static constexpr double ssl_temperature = 288.15;             // K (15 C)
    static constexpr double ssl_pressure    = 101325.0;           // Pa
    static constexpr double ssl_density     = 1.225;              // kg/m^3
    static constexpr double ssl_sonic_vel   = 340.294;            // m/s
    static constexpr double lapse_rate      = 0.0065;             // K/m (troposphere)
    static constexpr double tropopause_alt  = 11000.0;            // m
    static constexpr double tropopause_temp = 216.65;             // K
    static constexpr double gamma_air       = 1.4;                // specific heat ratio
    static constexpr double gas_constant_air = 287.058;           // J/(kg*K)
    static constexpr double gmr             = 0.0341631947;       // g*M/R (1/m)

    // Conversions
    static constexpr double ft_to_m         = 0.3048;
    static constexpr double m_to_ft         = 1.0 / 0.3048;
    static constexpr double nm_to_m         = 1852.0;             // nautical mile
    static constexpr double kt_to_mps       = 1852.0 / 3600.0;   // knots to m/s
    static constexpr double mps_to_kt       = 3600.0 / 1852.0;
    static constexpr double dbw_to_w        = 1.0;                // placeholder for clarity
};

// dB conversions
inline constexpr double db_to_linear(double db) noexcept {
    return std::pow(10.0, db / 10.0);
}

inline constexpr double linear_to_db(double linear) noexcept {
    return 10.0 * std::log10(linear);
}

inline constexpr double db_to_linear_power(double db) noexcept {
    return db_to_linear(db);
}

inline constexpr double db_to_linear_voltage(double db) noexcept {
    return std::pow(10.0, db / 20.0);
}

} // namespace xsf_math
