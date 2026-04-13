#include <xsf_math/xsf_math.hpp>
#include <cstdio>

using namespace xsf_math;

int main() {
    printf("=== Orbital Mechanics Example ===\n\n");

    // LEO satellite: 400km altitude circular orbit (ISS-like)
    orbital_elements leo;
    leo.semi_major_axis_m = constants::earth_radius_m + 400000.0;
    leo.eccentricity = 0.001;
    leo.inclination_rad = 51.6 * constants::deg_to_rad;  // ISS inclination
    leo.raan_rad = 0.0;
    leo.arg_periapsis_rad = 0.0;
    leo.true_anomaly_rad = 0.0;

    printf("LEO Satellite (ISS-like orbit):\n");
    printf("  Semi-major axis: %.0f km\n", leo.semi_major_axis_m / 1000.0);
    printf("  Period: %.1f min\n", leo.period() / 60.0);
    printf("  Periapsis alt: %.0f km\n", leo.periapsis_altitude() / 1000.0);
    printf("  Apoapsis alt:  %.0f km\n", leo.apoapsis_altitude() / 1000.0);
    printf("  Velocity: %.0f m/s\n\n", leo.velocity(leo.radius()));

    // GEO satellite
    orbital_elements geo;
    geo.semi_major_axis_m = 42164000.0;  // ~35,786 km altitude
    geo.eccentricity = 0.0;
    geo.inclination_rad = 0.0;
    geo.raan_rad = 0.0;
    geo.arg_periapsis_rad = 0.0;
    geo.true_anomaly_rad = 0.0;

    printf("GEO Satellite:\n");
    printf("  Altitude: %.0f km\n", geo.apoapsis_altitude() / 1000.0);
    printf("  Period: %.2f hours\n", geo.period() / 3600.0);
    printf("  Velocity: %.0f m/s\n\n", geo.velocity(geo.radius()));

    // Propagate LEO orbit
    printf("LEO orbit propagation (1 orbit):\n");
    printf("Time(min)  True_Anom(deg)  Radius(km)  Alt(km)\n");
    printf("---------  --------------  ----------  -------\n");

    double period = leo.period();
    for (double t = 0; t <= period; t += period / 12.0) {
        auto state = propagate_kepler(leo, t);
        double r = state.radius();
        double alt = r - constants::earth_radius_m;
        printf("%7.1f    %12.1f    %8.0f    %6.0f\n",
               t / 60.0,
               state.true_anomaly_rad * constants::rad_to_deg,
               r / 1000.0,
               alt / 1000.0);
    }

    // Kepler's equation solver test
    printf("\nKepler's equation test (e=0.5, M=1.0 rad):\n");
    double E = solve_kepler(1.0, 0.5);
    double nu = eccentric_to_true_anomaly(E, 0.5);
    printf("  E = %.6f rad (%.2f deg)\n", E, E * constants::rad_to_deg);
    printf("  nu = %.6f rad (%.2f deg)\n", nu, nu * constants::rad_to_deg);

    // LOS visibility check
    printf("\nLine-of-sight visibility test:\n");
    vec3 pos_a, vel_a, pos_b, vel_b;
    elements_to_state(leo, pos_a, vel_a);

    auto leo2 = propagate_kepler(leo, period / 4.0);  // 90 deg ahead
    elements_to_state(leo2, pos_b, vel_b);

    bool visible = los_visible(pos_a, pos_b);
    printf("  Sat A to Sat B (90 deg apart): %s\n", visible ? "VISIBLE" : "BLOCKED");

    auto leo3 = propagate_kepler(leo, period / 2.0);  // 180 deg (opposite side)
    elements_to_state(leo3, pos_b, vel_b);
    visible = los_visible(pos_a, pos_b);
    printf("  Sat A to Sat C (180 deg apart): %s\n", visible ? "VISIBLE" : "BLOCKED");

    return 0;
}
