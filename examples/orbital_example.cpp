#include <xsf_math/xsf_math.hpp>
#include <xsf_common/validation_artifacts.hpp>
#include <cstdio>
#include <cmath>

using namespace xsf_math;

int main(int argc, char** argv) {
    const auto cli = xsf::validation::parse_cli(argc, argv);
    xsf::validation::case_artifacts artifacts("orbital_example", cli);

    printf("=== Orbital Mechanics Example ===\n\n");

    // 近地轨道卫星：400km 高度圆轨道（类似 ISS）
    orbital_elements leo;
    leo.semi_major_axis_m = constants::earth_radius_m + 400000.0;
    leo.eccentricity = 0.001;
    leo.inclination_rad = 51.6 * constants::deg_to_rad;  // ISS 轨道倾角
    leo.raan_rad = 0.0;
    leo.arg_periapsis_rad = 0.0;
    leo.true_anomaly_rad = 0.0;

    printf("LEO Satellite (ISS-like orbit):\n");
    printf("  Semi-major axis: %.0f km\n", leo.semi_major_axis_m / 1000.0);
    printf("  Period: %.1f min\n", leo.period() / 60.0);
    printf("  Periapsis alt: %.0f km\n", leo.periapsis_altitude() / 1000.0);
    printf("  Apoapsis alt:  %.0f km\n", leo.apoapsis_altitude() / 1000.0);
    printf("  Velocity: %.0f m/s\n\n", leo.velocity(leo.radius()));

    // 地球同步轨道卫星
    orbital_elements geo;
    geo.semi_major_axis_m = 42164000.0;  // 约 35,786 km 高度
    geo.eccentricity = 0.0;
    geo.inclination_rad = 0.0;
    geo.raan_rad = 0.0;
    geo.arg_periapsis_rad = 0.0;
    geo.true_anomaly_rad = 0.0;

    printf("GEO Satellite:\n");
    printf("  Altitude: %.0f km\n", geo.apoapsis_altitude() / 1000.0);
    printf("  Period: %.2f hours\n", geo.period() / 3600.0);
    printf("  Velocity: %.0f m/s\n\n", geo.velocity(geo.radius()));

    // 推进近地轨道
    printf("LEO orbit propagation (1 orbit):\n");
    printf("Time(min)  True_Anom(deg)  Radius(km)  Alt(km)\n");
    printf("---------  --------------  ----------  -------\n");

    if (artifacts.enabled()) {
        artifacts.write_timeseries_header({
            "time_min", "true_anomaly_deg", "radius_km", "altitude_km"
        });
    }

    double period = leo.period();
    for (double t = 0; t <= period; t += period / 12.0) {
        auto state = propagate_kepler(leo, t);
        double r = state.radius();
        double alt = r - constants::earth_radius_m;
        artifacts.append_timeseries_row({
            xsf::validation::make_field("", t / 60.0, 3).value,
            xsf::validation::make_field("", state.true_anomaly_rad * constants::rad_to_deg, 3).value,
            xsf::validation::make_field("", r / 1000.0, 3).value,
            xsf::validation::make_field("", alt / 1000.0, 3).value
        });
        printf("%7.1f    %12.1f    %8.0f    %6.0f\n",
               t / 60.0,
               state.true_anomaly_rad * constants::rad_to_deg,
               r / 1000.0,
               alt / 1000.0);
    }

    // 开普勒方程求解测试
    printf("\nKepler's equation test (e=0.5, M=1.0 rad):\n");
    double E = solve_kepler(1.0, 0.5);
    double nu = eccentric_to_true_anomaly(E, 0.5);
    artifacts.append_event(0.0,
                           "kepler_solution",
                           "Solved Kepler equation for a representative eccentric case.",
                           {
                               xsf::validation::make_field("eccentric_anomaly_rad", E, 6),
                               xsf::validation::make_field("true_anomaly_rad", nu, 6)
                           });
    printf("  E = %.6f rad (%.2f deg)\n", E, E * constants::rad_to_deg);
    printf("  nu = %.6f rad (%.2f deg)\n", nu, nu * constants::rad_to_deg);

    // 视线可见性检查
    printf("\nLine-of-sight visibility test:\n");
    vec3 pos_a, vel_a, pos_b, vel_b;
    elements_to_state(leo, pos_a, vel_a);

    auto leo2 = propagate_kepler(leo, period / 4.0);  // 提前 90 度
    elements_to_state(leo2, pos_b, vel_b);

    bool visible = los_visible(pos_a, pos_b);
    const bool visible_quarter = visible;
    printf("  Sat A to Sat B (90 deg apart): %s\n", visible ? "VISIBLE" : "BLOCKED");

    auto leo3 = propagate_kepler(leo, period / 2.0);  // 180 度（对侧）
    elements_to_state(leo3, pos_b, vel_b);
    visible = los_visible(pos_a, pos_b);
    const bool visible_opposite = visible;
    artifacts.append_event(period / 60.0,
                           "line_of_sight",
                           "Evaluated line-of-sight visibility at quarter- and half-orbit separations.",
                           {
                               xsf::validation::make_field("quarter_orbit_visible", visible_quarter),
                               xsf::validation::make_field("half_orbit_visible", visible_opposite)
                           });
    printf("  Sat A to Sat C (180 deg apart): %s\n", visible ? "VISIBLE" : "BLOCKED");

    const bool passed =
        std::abs(leo.period() / 60.0 - 92.4) < 1.0 &&
        std::abs(geo.period() / 3600.0 - 23.93) < 0.5 &&
        !visible_quarter &&
        !visible_opposite;
    const std::string failure_reason = passed
        ? std::string()
        : "Orbital propagation or line-of-sight checks drifted away from the expected reference behavior";

    artifacts.write_metrics({
        xsf::validation::make_field("leo_period_min", leo.period() / 60.0, 3),
        xsf::validation::make_field("geo_period_hr", geo.period() / 3600.0, 3),
        xsf::validation::make_field("kepler_e_rad", E, 6),
        xsf::validation::make_field("kepler_nu_rad", nu, 6),
        xsf::validation::make_field("quarter_orbit_visible", visible_quarter),
        xsf::validation::make_field("half_orbit_visible", visible_opposite)
    });
    artifacts.write_summary(
        passed,
        "Validate orbital propagation, representative Kepler solving, and line-of-sight blocking behavior.",
        "LEO and GEO periods should stay close to known references, and same-orbit satellites at quarter and half separation should remain Earth-blocked in this setup.",
        passed
            ? "Propagation periods matched expected magnitudes and the visibility checks stayed consistent with the configured geometry."
            : "At least one orbital reference metric or visibility expectation drifted outside the accepted range.",
        {
            xsf::validation::make_field("leo_altitude_km", 400.0, 1),
            xsf::validation::make_field("geo_altitude_km", geo.apoapsis_altitude() / 1000.0, 3),
            xsf::validation::make_field("leo_inclination_deg", leo.inclination_rad * constants::rad_to_deg, 3)
        },
        {
            xsf::validation::make_field("leo_period_min", leo.period() / 60.0, 3),
            xsf::validation::make_field("geo_period_hr", geo.period() / 3600.0, 3),
            xsf::validation::make_field("kepler_e_rad", E, 6),
            xsf::validation::make_field("kepler_nu_rad", nu, 6),
            xsf::validation::make_field("quarter_orbit_visible", visible_quarter),
            xsf::validation::make_field("half_orbit_visible", visible_opposite)
        },
        failure_reason);

    return (cli.strict && !passed) ? 1 : 0;
}
