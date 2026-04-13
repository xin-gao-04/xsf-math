#pragma once

#include "../core/constants.hpp"
#include "../core/atmosphere.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace xsf_math {

// Aerodynamic force computation

// Basic aerodynamic coefficients
struct aero_coefficients {
    double cl = 0.0;     // lift coefficient
    double cd = 0.0;     // drag coefficient
    double cy = 0.0;     // side force coefficient
};

// Basic aerodynamic state
struct aero_state {
    double altitude_m     = 0.0;
    double speed_mps      = 0.0;
    double mach           = 0.0;
    double dynamic_pressure_pa = 0.0;
    double air_density    = 0.0;
    double angle_of_attack_rad = 0.0;  // alpha
    double sideslip_angle_rad  = 0.0;  // beta

    // Compute from altitude and speed
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

// Aerodynamic forces in body (ECS) frame
struct aero_forces {
    double drag_n   = 0.0;  // along -X (opposing forward motion)
    double side_n   = 0.0;  // along Y (right wing)
    double normal_n = 0.0;  // along Z (down)
    double lift_n   = 0.0;  // perpendicular to velocity (derived)

    // Total force magnitude
    double total() const {
        return std::sqrt(drag_n*drag_n + side_n*side_n + normal_n*normal_n);
    }
};

// 2D aerodynamic model (lift + drag, parabolic drag polar)
// Cd = Cd0 + CL^2 / (pi * AR * e)
struct aero_2d {
    double ref_area_m2   = 1.0;    // reference area (wing area)
    double aspect_ratio  = 6.0;    // AR = wingspan^2 / area
    double oswald_factor = 0.8;    // e (Oswald efficiency)
    double cl_max        = 1.5;    // maximum CL before stall
    double lift_curve_slope = 0.1; // dCL/dalpha (per degree), ~2*pi/rad

    // Zero-lift drag coefficient (can be Mach-dependent)
    double cd0 = 0.02;

    // Drag coefficient table: Cd0 vs Mach (optional)
    std::vector<double> cd0_mach_table;   // Mach breakpoints
    std::vector<double> cd0_values;       // Cd0 at each Mach

    // Induced drag factor K = 1 / (pi * AR * e)
    double induced_drag_factor() const {
        return 1.0 / (constants::pi * aspect_ratio * oswald_factor);
    }

    // Get Cd0 for given Mach number (table lookup or constant)
    double get_cd0(double mach) const {
        if (cd0_mach_table.empty() || cd0_values.empty()) return cd0;
        return table_lookup(cd0_mach_table, cd0_values, mach);
    }

    // Compute forces
    aero_forces compute(const aero_state& state, double requested_g_normal = 0.0,
                         double requested_g_lateral = 0.0) const {
        double q = state.dynamic_pressure_pa;
        double S = ref_area_m2;
        double qS = q * S;

        if (qS < 1e-10) return {};

        // CL from requested normal force (or from angle of attack)
        double cl;
        if (std::abs(requested_g_normal) > 1e-10) {
            // Compute CL needed for requested force
            double weight_equivalent = requested_g_normal * constants::gravity_mps2;
            cl = weight_equivalent / qS;
        } else {
            cl = lift_curve_slope * (state.angle_of_attack_rad * constants::rad_to_deg);
        }

        // Clamp CL to max
        cl = clamp(cl, -cl_max, cl_max);

        // Drag polar
        double cd0_m = get_cd0(state.mach);
        double K = induced_drag_factor();
        double cd_total = cd0_m + K * cl * cl;

        aero_forces f;
        f.drag_n   = qS * cd_total;
        f.lift_n   = qS * cl;
        f.normal_n = -f.lift_n;  // normal force in body Z (down)

        // Lateral force (side force)
        if (std::abs(requested_g_lateral) > 1e-10) {
            double cy = requested_g_lateral * constants::gravity_mps2 / qS;
            f.side_n = qS * cy;
        }

        return f;
    }

    // Maximum available g-load
    double max_g_available(const aero_state& state, double mass_kg) const {
        if (mass_kg <= 0.0) return 0.0;
        double qS = state.dynamic_pressure_pa * ref_area_m2;
        double max_force = qS * cl_max;
        return max_force / (mass_kg * constants::gravity_mps2);
    }

    // Specific excess power (Ps)
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

// Fuel consumption model
struct fuel_model {
    double initial_fuel_kg = 100.0;
    double current_fuel_kg = 100.0;

    // Specific fuel consumption: kg/(N*s)
    double sfc = 0.0001;  // thrust-specific

    // Consume fuel for given thrust and time
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

// Simple rocket motor model
struct rocket_motor {
    double thrust_n     = 10000.0;   // nominal thrust
    double burn_time_s  = 10.0;      // total burn time
    double isp_s        = 250.0;     // specific impulse
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

    // Fuel consumption rate (kg/s)
    double fuel_rate() const {
        if (!is_burning()) return 0.0;
        return thrust_n / (isp_s * constants::gravity_mps2);
    }
};

// Stall detection
inline bool is_stalled(double aoa_rad, double critical_aoa_rad = 15.0 * constants::deg_to_rad) {
    return std::abs(aoa_rad) > critical_aoa_rad;
}

// Flight envelope check
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
