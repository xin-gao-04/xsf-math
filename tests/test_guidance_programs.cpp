#include <xsf_behavior/xsf_behavior.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace xsf_math;

namespace {

guidance_program_state make_state(const vec3& position_wcs,
                                  const vec3& velocity_wcs,
                                  double current_time_s = 1.0,
                                  double end_time_s = 1.2,
                                  double last_update_time_s = 0.8) {
    guidance_program_state state;
    state.vehicle = flight_kinematic_state::from_velocity(position_wcs, velocity_wcs);
    state.vehicle.heading_rad = 0.0;
    state.vehicle.pitch_rad = 0.0;
    state.vehicle.roll_rad = 0.0;
    state.current_time_s = current_time_s;
    state.end_time_s = end_time_s;
    state.last_update_time_s = last_update_time_s;
    return state;
}

bool approx_rel(double a, double b, double tol = 0.05) {
    if (std::abs(b) < 1e-12) return std::abs(a) < tol;
    return std::abs((a - b) / b) < tol;
}

} // namespace

void test_legacy_program_guidance_and_bias() {
    printf("  legacy_program_guidance_and_bias... ");
    auto state = make_state({0.0, 0.0, -1000.0}, {300.0, 0.0, 0.0});
    state.aimpoint_is_valid = true;
    state.aimpoint_position_wcs = {5000.0, 2000.0, -1000.0};
    state.target_velocity_wcs = {0.0, 0.0, 0.0};

    legacy_guidance_program program;
    guidance_phase_options phase;
    phase.pn_gain = 0.0;
    phase.vp_gain = 10.0;
    phase.gee_bias = 1.0;

    auto result = program.compute(state, phase);
    assert(result.status == guidance_program_status::continue_running);
    assert(result.commands.accel_cmd_ecs.y > 0.0);
    assert(result.commands.accel_cmd_ecs.z < 0.0);
    printf("OK\n");
}

void test_altitude_program_agl() {
    printf("  altitude_program_agl... ");
    auto state = make_state({0.0, 0.0, -100.0}, {220.0, 0.0, 0.0});
    state.terrain_enabled = true;
    state.terrain_height_m = 50.0;

    altitude_guidance_program program;
    program.commanded_altitude_m = 200.0;
    program.commanded_altitude_is_agl = true;

    guidance_phase_options phase;
    auto result = program.compute(state, phase);
    assert(result.commands.accel_cmd_ecs.z < 0.0);
    printf("OK\n");
}

void test_intercept_program_switching() {
    printf("  intercept_program_switching... ");
    auto state = make_state({0.0, 0.0, -1000.0}, {500.0, 0.0, 0.0});
    state.aimpoint_is_valid = true;
    state.aimpoint_position_wcs = {5000.0, 10.0, -1000.0};
    state.target_velocity_wcs = {0.0, 250.0, 0.0};

    guidance_phase_options phase;
    phase.pn_gain = 3.0;
    phase.vp_gain = 10.0;

    intercept_guidance_program pursuit_only;
    pursuit_only.pro_nav_gain = 0.0;
    auto pursuit_result = pursuit_only.compute(state, phase);

    intercept_guidance_program forced_pursuit;
    forced_pursuit.cos_switch_angle = 1.1;
    auto forced_pursuit_result = forced_pursuit.compute(state, phase);

    intercept_guidance_program forced_pn;
    forced_pn.cos_switch_angle = 0.5;
    auto forced_pn_result = forced_pn.compute(state, phase);

    assert(approx_rel(forced_pursuit_result.commands.accel_cmd_ecs.y, pursuit_result.commands.accel_cmd_ecs.y, 0.001));
    assert(std::abs(forced_pn_result.commands.accel_cmd_ecs.y - pursuit_result.commands.accel_cmd_ecs.y) > 1.0e-3 ||
           std::abs(forced_pn_result.commands.accel_cmd_ecs.z - pursuit_result.commands.accel_cmd_ecs.z) > 1.0e-3);
    printf("OK\n");
}

void test_legacy_flight_path_angle_program() {
    printf("  legacy_flight_path_angle_program... ");
    auto state = make_state({0.0, 0.0, -500.0}, {250.0, 0.0, 0.0});
    state.vehicle.flight_path_rad = 0.0;

    legacy_flight_path_angle_program program;
    program.commanded_flight_path_angle_rad = 10.0 * constants::deg_to_rad;

    guidance_phase_options phase;
    auto result = program.compute(state, phase);
    assert(result.status == guidance_program_status::continue_running);
    assert(result.commands.accel_cmd_ecs.z < 0.0);

    state.vehicle.flight_path_rad = 10.0 * constants::deg_to_rad;
    auto complete_result = program.compute(state, phase);
    assert(complete_result.status == guidance_program_status::complete);
    printf("OK\n");
}

void test_gravity_bias_program() {
    printf("  gravity_bias_program... ");
    auto state = make_state({0.0, 0.0, -800.0}, {200.0, 0.0, 0.0});
    gravity_bias_program program;
    auto result = program.compute(state, guidance_phase_options{});
    assert(result.commands.accel_cmd_ecs.z < 0.0);
    printf("OK\n");
}

void test_gravity_turn_program() {
    printf("  gravity_turn_program... ");
    auto state = make_state({0.0, 0.0, -800.0}, {200.0, 0.0, 0.0});
    gravity_turn_program program;
    auto result = program.compute(state, guidance_phase_options{});
    assert(result.commands.accel_cmd_ecs.z == 0.0);
    printf("OK\n");
}

int main() {
    printf("=== Guidance Program Tests ===\n");
    test_legacy_program_guidance_and_bias();
    test_altitude_program_agl();
    test_intercept_program_switching();
    test_legacy_flight_path_angle_program();
    test_gravity_bias_program();
    test_gravity_turn_program();
    printf("All guidance program tests passed.\n");
    return 0;
}
