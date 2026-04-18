#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cstdio>

using namespace xsf_math;

namespace {

bool approx(double a, double b, double tol = 1.0e-5) {
    return std::abs(a - b) <= tol;
}

void test_quaternion_roundtrip() {
    printf("  quaternion_roundtrip... ");
    euler_angles e{35.0 * constants::deg_to_rad, -10.0 * constants::deg_to_rad, 15.0 * constants::deg_to_rad};
    quaternion q = quaternion_wcs_to_ecs(e);
    euler_angles e2 = quaternion_to_euler(q);
    assert(approx(e.heading_rad, e2.heading_rad, 1.0e-4));
    assert(approx(e.pitch_rad, e2.pitch_rad, 1.0e-4));
    assert(approx(e.roll_rad, e2.roll_rad, 1.0e-4));
    printf("OK\n");
}

void test_wgs84_roundtrip() {
    printf("  wgs84_roundtrip... ");
    lla geo{31.2304 * constants::deg_to_rad, 121.4737 * constants::deg_to_rad, 35.0};
    vec3 ecef = lla_to_ecef(geo);
    lla geo2 = ecef_to_lla(ecef);
    assert(approx(geo.lat_rad, geo2.lat_rad, 1.0e-6));
    assert(approx(geo.lon_rad, geo2.lon_rad, 1.0e-6));
    assert(std::abs(geo.alt_m - geo2.alt_m) < 1.0);
    printf("OK\n");
}

void test_ekf_imm_and_rcs() {
    printf("  ekf_imm_and_rcs... ");
    extended_kalman_filter_6state ekf;
    spherical_measurement z0{1000.0, 0.1, -0.05};
    ekf.update_spherical(0.0, z0);
    ekf.update_spherical(1.0, {980.0, 0.09, -0.045});
    assert(ekf.position().magnitude() > 0.0);

    imm_filter_6state imm;
    imm.active_model_count = 2;
    imm.set_process_noise_profile(0, 0.2, 0.2, 0.2);
    imm.set_process_noise_profile(1, 5.0, 5.0, 5.0);
    imm.init(0.0, {0.0, 0.0, 0.0}, {100.0, 0.0, 0.0});
    imm.predict(1.0);
    imm.update(1.0, {95.0, 5.0, 0.0});
    assert(imm.fused_position().magnitude() > 0.0);

    sphere_rcs sphere{0.5};
    flat_plate_rcs plate{2.0, 1.0};
    assert(sphere.evaluate(0.0, 0.0, 10.0e9).m2 > 0.0);
    assert(plate.evaluate(0.0, 0.0, 10.0e9).m2 > 0.0);
    printf("OK\n");
}

void test_lambert_and_sgp4_lite() {
    printf("  lambert_and_sgp4_lite... ");
    vec3 r1{7000e3, 0.0, 0.0};
    vec3 r2{0.0, 8000e3, 0.0};
    auto lambert = solve_lambert(r1, r2, 1800.0);
    assert(lambert.valid);
    assert(lambert.departure_velocity_eci.magnitude() > 0.0);

    tle_record tle;
    tle.inclination_rad = 51.6 * constants::deg_to_rad;
    tle.raan_rad = 20.0 * constants::deg_to_rad;
    tle.eccentricity = 0.0002;
    tle.arg_perigee_rad = 0.0;
    tle.mean_anomaly_rad = 0.0;
    tle.mean_motion_rev_per_day = 15.5;
    vec3 pos{}, vel{};
    propagate_sgp4_lite(tle, tle.epoch_julian_date + 0.01, pos, vel);
    assert(pos.magnitude() > constants::earth_radius_m);
    assert(vel.magnitude() > 1000.0);
    printf("OK\n");
}

void test_ew_sensor_and_behavior_helpers() {
    printf("  ew_sensor_and_behavior_helpers... ");
    rgpo_profile rgpo;
    auto false_state = apply_rgpo(rgpo, 2.0);
    assert(false_state.apparent_range_offset_m > 0.0);

    rwr_receiver rwr;
    auto contact = rwr.evaluate(1.0e-11, 2000.0, 50000.0);
    assert(contact.detected);

    sensor_scheduler scheduler;
    scheduler.params.priority_policy = sensor_priority_policy::threat_weighted;
    scheduler.search_list.push_back({0, 1.0});
    scheduler.search_list.push_back({1, 5.0});
    auto cmd = scheduler.select(0.0, sensor_mode::search);
    assert(cmd.valid);
    assert(cmd.target_index == 1);

    track_manager manager;
    track_initiator initiator;
    detection_sample a;
    a.measurement.position = {0.0, 0.0, 0.0};
    a.measurement.meas_noise[0] = a.measurement.meas_noise[1] = a.measurement.meas_noise[2] = 25.0;
    a.measurement.target_index = 3;
    a.sim_time_s = 0.0;

    detection_sample b;
    b.measurement.position = {100.0, 0.0, 0.0};
    b.measurement.meas_noise[0] = b.measurement.meas_noise[1] = b.measurement.meas_noise[2] = 25.0;
    b.measurement.target_index = 3;
    b.sim_time_s = 1.0;
    auto seed = initiator.seed_from_two_points(a, b);
    assert(seed.valid);
    int track_id = initiator.start_track(manager, seed, 1.0);
    assert(track_id > 0);

    weapon_assignment_controller wta;
    auto result = wta.assign({{1, true}, {2, true}},
                             {{101, 2.0}, {102, 1.0}},
                             {{0.2, 0.8}, {0.9, 0.1}});
    assert(!result.assignments.empty());
    assert(result.total_score > 0.0);

    engagement_controller engagement;
    engagement_context ctx;
    ctx.weapon_launched = true;
    ctx.geom.weapon_pos = {0.0, 0.0, 0.0};
    ctx.geom.weapon_vel = {300.0, 0.0, 0.0};
    ctx.geom.target_pos = {1000.0, 0.0, 0.0};
    ctx.geom.target_vel = {0.0, 0.0, 0.0};
    ctx.fuze_in.target_pos = ctx.geom.target_pos;
    ctx.fuze_in.weapon_pos = ctx.geom.weapon_pos;
    ctx.fuze_in.target_vel = ctx.geom.target_vel;
    ctx.fuze_in.weapon_vel = ctx.geom.weapon_vel;
    ctx.fuze_in.dt_s = 0.1;
    ctx.has_weapon_attitude = true;
    ctx.weapon_attitude = {};
    auto ecmd = engagement.update(ctx);
    (void)ecmd.guidance_channel_cmd;
    assert(ecmd.phase == engagement_phase::terminal || ecmd.phase == engagement_phase::midcourse);

    orbital_maneuver_planner planner;
    vec3 r1{7000e3, 0.0, 0.0};
    vec3 r2{0.0, 8000e3, 0.0};
    auto plan = planner.plan_lambert_intercept(r1, {0.0, 7.5e3, 0.0}, r2, 1800.0);
    assert(!plan.steps.empty());
    printf("OK\n");
}

void test_multisensor_and_tracking_extensions() {
    printf("  multisensor_and_tracking_extensions... ");
    cell_averaging_cfar cfar;
    cfar.guard_cells = 1;
    cfar.training_cells = 2;
    auto cfar_res = cfar.evaluate({1.0, 1.1, 1.0, 12.0, 1.2, 1.0, 1.1, 1.0, 1.0});
    assert(!cfar_res.detections.empty());

    auto doppler = evaluate_doppler_ambiguity(250.0, 10.0e9, 2000.0);
    assert(std::abs(doppler.folded_frequency_hz) <= 1000.0 + 1.0e-6);

    infrared_detector ir;
    auto ir_res = ir.evaluate({2.0, 800.0, 0.95}, {}, 5000.0);
    assert(ir_res.snr_linear > 0.0);

    esm_receiver esm;
    auto esm_res = esm.evaluate({1.0e-10, 10.0e9, 3000.0, 2.0e-6, 2.0}, 0.5);
    assert(esm_res.intercept_probability > 0.0);

    active_sonar_equation active_sonar;
    auto sonar_res = active_sonar.evaluate(3000.0);
    assert(sonar_res.pd >= 0.0);

    std::vector<track_state> tracks = {
        {1, {0.0, 0.0, 0.0}, {}, {25.0, 25.0, 25.0}},
        {2, {1000.0, 0.0, 0.0}, {}, {25.0, 25.0, 25.0}}
    };
    std::vector<detection> dets = {
        {{10.0, 0.0, 0.0}, {25.0, 25.0, 25.0}},
        {{995.0, 5.0, 0.0}, {25.0, 25.0, 25.0}}
    };

    gnn_associator gnn;
    auto gnn_res = gnn.associate(tracks, dets);
    assert(gnn_res.assignments.size() == tracks.size());

    jpda_associator jpda;
    auto jpda_res = jpda.associate(tracks, dets);
    assert(jpda_res.hypothesis_count > 0);

    mht_associator mht;
    auto branches = mht.expand(tracks, dets);
    assert(!branches.empty());

    ukf_filter_6state ukf;
    ukf.update_spherical(0.0, {1000.0, 0.0, 0.0});
    ukf.update_position(1.0, {950.0, 10.0, 0.0}, 20.0);
    assert(ukf.position().magnitude() > 0.0);

    particle_filter_6state pf;
    pf.init(64, {0.0, 0.0, 0.0}, {100.0, 0.0, 0.0});
    pf.predict(1.0);
    pf.update_position({95.0, 2.0, 0.0}, 20.0);
    assert(pf.estimate_position().magnitude() > 0.0);
    printf("OK\n");
}

void test_dynamics_navigation_and_support_extensions() {
    printf("  dynamics_navigation_and_support_extensions... ");
    rigid_body_state rb;
    rb.mass_kg = 100.0;
    six_dof_integrator integrator;
    integrator.step(rb, {{500.0, 0.0, 0.0}, {0.0, 0.0, 5.0}, {}, 0.1}, 0.5);
    assert(rb.velocity_wcs.magnitude() > 0.0);

    gps_ins_state nav;
    gps_ins_loose_coupler coupler;
    imu_sample imu{};
    imu.specific_force_body_mps2 = {0.0, 0.0, -constants::gravity_mps2};
    imu.dt_s = 0.1;
    coupler.propagate(nav, imu);
    coupler.update(nav, {true, {10.0, 0.0, 0.0}, {100.0, 0.0, 0.0}, 5.0, 1.0});
    assert(nav.position_wcs.magnitude() >= 0.0);

    std::vector<vec3> sats = {
        {20200e3, 0.0, 0.0},
        {0.0, 20200e3, 0.0},
        {0.0, 0.0, 20200e3},
        {-20200e3, 0.0, 0.0},
        {0.0, -20200e3, 0.0}
    };
    auto visibility = estimate_gps_visibility(sats, {constants::earth_equatorial_radius_m, 0.0, 0.0});
    assert(visibility.satellites_visible >= 1);

    datalink_budget link;
    auto contact = link.evaluate({}, {}, 50000.0, 1.0e9, 1024.0);
    assert(contact.throughput_bps > 0.0);

    tdma_scheduler tdma;
    auto slots = tdma.allocate({{1, 4096.0, 3.0}, {2, 2048.0, 1.0}}, 1.0e6);
    assert(!slots.empty());

    dubins_planner dubins;
    auto path = dubins.plan({0.0, 0.0, 0.0}, {5000.0, 2000.0, 0.5},
                            {{{2500.0, 1000.0, 0.0}, 500.0}});
    assert(path.valid);
    printf("OK\n");
}

void test_reentry_and_lethality_extensions() {
    printf("  reentry_and_lethality_extensions... ");
    reentry_state entry;
    reentry_propagator reentry;
    auto step = reentry.step(entry, 0.2);
    assert(step.dynamic_pressure_pa >= 0.0);

    fragment_pattern pattern;
    auto fragment = evaluate_fragment_cloud(pattern, 20.0, 5.0);
    assert(fragment.hit_probability > 0.0);

    vulnerability_model vulnerability;
    vulnerability.components = {
        {"engine", 2.0, 1.0, 0.7, 0.9},
        {"control", 1.0, 1.0, 0.5, 0.6}
    };
    auto vuln = vulnerability.evaluate(fragment.areal_density_per_m2, 0.8);
    assert(vuln.cumulative_kill_probability > 0.0);

    pip_guidance pip;
    engagement_geometry geom;
    geom.weapon_pos = {0.0, 0.0, 0.0};
    geom.weapon_vel = {300.0, 0.0, 0.0};
    geom.target_pos = {2000.0, 200.0, 0.0};
    geom.target_vel = {50.0, 0.0, 0.0};
    auto pip_cmd = pip.compute_accel(geom);
    assert(pip_cmd.magnitude() >= 0.0);
    printf("OK\n");
}

}  // namespace

int main() {
    printf("=== Roadmap Feature Tests ===\n");
    test_quaternion_roundtrip();
    test_wgs84_roundtrip();
    test_ekf_imm_and_rcs();
    test_lambert_and_sgp4_lite();
    test_ew_sensor_and_behavior_helpers();
    test_multisensor_and_tracking_extensions();
    test_dynamics_navigation_and_support_extensions();
    test_reentry_and_lethality_extensions();
    printf("All roadmap feature tests passed.\n");
    return 0;
}
