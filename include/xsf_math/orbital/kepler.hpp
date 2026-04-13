#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Keplerian orbital mechanics

// Gravitational parameter (mu) for Earth
constexpr double mu_earth = 3.986004418e14;  // m^3/s^2

// Classical orbital elements
struct orbital_elements {
    double semi_major_axis_m;  // a
    double eccentricity;       // e (0=circle, 0<e<1=ellipse, 1=parabolic, >1=hyperbolic)
    double inclination_rad;    // i
    double raan_rad;           // Omega (right ascension of ascending node)
    double arg_periapsis_rad;  // omega (argument of periapsis)
    double true_anomaly_rad;   // nu (true anomaly)

    // Orbital period (seconds)
    double period() const {
        if (eccentricity >= 1.0 || semi_major_axis_m <= 0.0) return 1e20;
        return 2.0 * constants::pi * std::sqrt(
            std::pow(semi_major_axis_m, 3) / mu_earth);
    }

    // Mean motion (rad/s)
    double mean_motion() const {
        if (semi_major_axis_m <= 0.0) return 0.0;
        return std::sqrt(mu_earth / std::pow(semi_major_axis_m, 3));
    }

    // Altitude at periapsis and apoapsis (from Earth center)
    double periapsis_radius() const {
        return semi_major_axis_m * (1.0 - eccentricity);
    }
    double apoapsis_radius() const {
        if (eccentricity >= 1.0) return 1e20;
        return semi_major_axis_m * (1.0 + eccentricity);
    }

    // Altitude above Earth surface
    double periapsis_altitude() const { return periapsis_radius() - constants::earth_radius_m; }
    double apoapsis_altitude() const { return apoapsis_radius() - constants::earth_radius_m; }

    // Orbital velocity at current position
    double velocity(double radius_m) const {
        // Vis-viva: v^2 = mu * (2/r - 1/a)
        return std::sqrt(mu_earth * (2.0/radius_m - 1.0/semi_major_axis_m));
    }

    // Radius at current true anomaly
    double radius() const {
        double p = semi_major_axis_m * (1.0 - eccentricity * eccentricity);
        return p / (1.0 + eccentricity * std::cos(true_anomaly_rad));
    }
};

// Solve Kepler's equation: M = E - e*sin(E)
// Given mean anomaly M and eccentricity e, find eccentric anomaly E
inline double solve_kepler(double M_rad, double eccentricity, int max_iter = 30) {
    // Initial guess
    double E = M_rad;
    if (eccentricity > 0.8) {
        E = constants::pi;
    }

    // Newton-Raphson iteration
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

// Convert eccentric anomaly to true anomaly
inline double eccentric_to_true_anomaly(double E_rad, double eccentricity) {
    double half_E = E_rad / 2.0;
    double factor = std::sqrt((1.0 + eccentricity) / (1.0 - eccentricity));
    return 2.0 * std::atan2(factor * std::sin(half_E), std::cos(half_E));
}

// Convert true anomaly to eccentric anomaly
inline double true_to_eccentric_anomaly(double nu_rad, double eccentricity) {
    return 2.0 * std::atan2(
        std::sqrt(1.0 - eccentricity) * std::sin(nu_rad / 2.0),
        std::sqrt(1.0 + eccentricity) * std::cos(nu_rad / 2.0));
}

// Mean anomaly from eccentric anomaly
inline double eccentric_to_mean_anomaly(double E_rad, double eccentricity) {
    return E_rad - eccentricity * std::sin(E_rad);
}

// Propagate orbit by time delta (two-body Keplerian)
inline orbital_elements propagate_kepler(const orbital_elements& oe, double dt_s) {
    double n = oe.mean_motion();

    // Current mean anomaly
    double E0 = true_to_eccentric_anomaly(oe.true_anomaly_rad, oe.eccentricity);
    double M0 = eccentric_to_mean_anomaly(E0, oe.eccentricity);

    // New mean anomaly
    double M = std::fmod(M0 + n * dt_s, constants::two_pi);
    if (M < 0) M += constants::two_pi;

    // Solve for new eccentric anomaly
    double E = solve_kepler(M, oe.eccentricity);

    // New true anomaly
    double nu = eccentric_to_true_anomaly(E, oe.eccentricity);

    orbital_elements result = oe;
    result.true_anomaly_rad = nu;
    return result;
}

// Convert orbital elements to ECI position and velocity
inline void elements_to_state(const orbital_elements& oe, vec3& pos, vec3& vel) {
    double e = oe.eccentricity;
    double a = oe.semi_major_axis_m;
    double nu = oe.true_anomaly_rad;

    double p = a * (1.0 - e*e);
    double r = p / (1.0 + e * std::cos(nu));

    // Position in orbital plane
    double x_orb = r * std::cos(nu);
    double y_orb = r * std::sin(nu);

    // Velocity in orbital plane
    double h = std::sqrt(mu_earth * p);
    double vx_orb = -(mu_earth / h) * std::sin(nu);
    double vy_orb =  (mu_earth / h) * (e + std::cos(nu));

    // Rotation from orbital plane to ECI
    double ci = std::cos(oe.inclination_rad);
    double si = std::sin(oe.inclination_rad);
    double cO = std::cos(oe.raan_rad);
    double sO = std::sin(oe.raan_rad);
    double cw = std::cos(oe.arg_periapsis_rad);
    double sw = std::sin(oe.arg_periapsis_rad);

    // Rotation matrix elements
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

// Line-of-sight visibility between two satellites
// Returns true if the line between them does not intersect Earth
inline bool los_visible(const vec3& pos_a, const vec3& pos_b) {
    vec3 d = pos_b - pos_a;
    double a = d.magnitude_sq();
    double b = 2.0 * pos_a.dot(d);
    double c = pos_a.magnitude_sq() - constants::earth_radius_m * constants::earth_radius_m;

    double discriminant = b*b - 4.0*a*c;
    if (discriminant < 0.0) return true;  // no intersection

    double sqrt_disc = std::sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2.0*a);
    double t2 = (-b + sqrt_disc) / (2.0*a);

    // Intersection within segment [0,1] means blocked
    return !(t1 >= 0.0 && t1 <= 1.0) && !(t2 >= 0.0 && t2 <= 1.0);
}

// Atmospheric drag deceleration (simplified)
inline double atmospheric_drag_accel(double altitude_m, double speed_mps,
                                      double drag_coeff_cd, double area_m2,
                                      double mass_kg) {
    // Exponential atmosphere model for density at orbital altitudes
    constexpr double rho0 = 1.225;
    constexpr double H = 8500.0;  // scale height (m)
    double rho = rho0 * std::exp(-altitude_m / H);

    double drag_force = 0.5 * rho * speed_mps * speed_mps * drag_coeff_cd * area_m2;
    return (mass_kg > 0.0) ? drag_force / mass_kg : 0.0;
}

} // namespace xsf_math
