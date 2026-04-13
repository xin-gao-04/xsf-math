#include <xsf_behavior/xsf_behavior.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace xsf_math;

static bool approx_rel(double a, double b, double tol = 0.05) {
    if (std::abs(b) < 1e-12) return std::abs(a) < tol;
    return std::abs((a - b) / b) < tol;
}

// 验证从位置/速度快速构造飞行状态时，派生量方向和符号约定正确。
void test_flight_state() {
    printf("  flight_state... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -2000.0}, {200.0, 0.0, -20.0});
    assert(approx_rel(state.altitude_m, 2000.0, 0.001));
    assert(state.true_airspeed_mps > 200.0);
    assert(state.vertical_speed_mps > 0.0);
    assert(state.dynamic_pressure_pa > 0.0);
    printf("OK\n");
}

// 拉升控制器应输出正俯仰、正爬升率，并给出不小于 1g 的法向过载。
void test_pull_up_controller() {
    printf("  pull_up_controller... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -1000.0}, {220.0, 0.0, 0.0});
    state.pitch_rad = 0.0;

    pull_up_target target;
    target.target_altitude_m = 3000.0;
    target.target_climb_angle_rad = 12.0 * constants::deg_to_rad;

    flight_control_limits limits;
    pull_up_controller controller;
    auto command = controller.compute(state, target, limits, 0.1);

    assert(command.valid);
    assert(command.commanded_pitch_rad > 0.0);
    assert(command.commanded_vertical_speed_mps > 0.0);
    assert(command.commanded_normal_load_factor_g >= 1.0);
    printf("OK\n");
}

// 协调转弯控制器应根据航向误差输出同号滚转和航向率。
void test_coordinated_turn_controller() {
    printf("  coordinated_turn_controller... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -2000.0}, {180.0, 0.0, 0.0});
    state.heading_rad = 0.0;
    state.roll_rad = 0.0;

    coordinated_turn_target target;
    target.target_heading_rad = 45.0 * constants::deg_to_rad;

    flight_control_limits limits;
    coordinated_turn_controller controller;
    auto command = controller.compute(state, target, limits, 0.2);

    assert(command.valid);
    assert(command.commanded_roll_rad > 0.0);
    assert(command.commanded_heading_rate_rad_s > 0.0);
    assert(command.commanded_normal_load_factor_g > 1.0);
    printf("OK\n");
}

// 下降控制器应输出负俯仰和负垂向速度。
void test_descent_controller() {
    printf("  descent_controller... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -3000.0}, {160.0, 0.0, 0.0});
    state.pitch_rad = 2.0 * constants::deg_to_rad;

    descent_target target;
    target.target_altitude_m = 1000.0;
    target.target_descent_angle_rad = 4.0 * constants::deg_to_rad;

    flight_control_limits limits;
    descent_controller controller;
    auto command = controller.compute(state, target, limits, 0.2);

    assert(command.valid);
    assert(command.commanded_pitch_rad < 0.0);
    assert(command.commanded_vertical_speed_mps < 0.0);
    printf("OK\n");
}

// 控制器输出需要遵守单步速率和姿态限制。
void test_control_limits() {
    printf("  control_limits... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -1500.0}, {120.0, 0.0, 0.0});
    coordinated_turn_target target;
    target.target_heading_rad = constants::pi;
    flight_control_limits limits;
    limits.max_roll_angle_rad = 20.0 * constants::deg_to_rad;
    limits.max_roll_rate_rad_s = 10.0 * constants::deg_to_rad;

    coordinated_turn_controller controller;
    auto command = controller.compute(state, target, limits, 0.1);
    assert(command.valid);
    assert(command.saturated);
    assert(command.commanded_roll_rad <= 1.0 * constants::deg_to_rad + limits.max_roll_rate_rad_s * 0.1);
    printf("OK\n");
}

// 当速度或动压不足时，控制器应拒绝输出有效控制命令。
void test_low_energy_rejection() {
    printf("  low_energy_rejection... ");
    auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -1000.0}, {20.0, 0.0, 0.0});
    pull_up_target target;
    target.target_altitude_m = 2000.0;
    flight_control_limits limits;

    pull_up_controller controller;
    auto command = controller.compute(state, target, limits, 0.1);
    assert(!command.valid);
    assert(command.status == flight_command_status::speed_limited);
    printf("OK\n");
}

int main() {
    printf("=== Flight Module Tests ===\n");
    test_flight_state();
    test_pull_up_controller();
    test_coordinated_turn_controller();
    test_descent_controller();
    test_control_limits();
    test_low_energy_rejection();
    printf("All flight tests passed.\n");
    return 0;
}
