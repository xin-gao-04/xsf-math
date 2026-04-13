#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <cctype>

using namespace xsf_math;

namespace {

struct case_block {
    std::map<std::string, std::string> values;
};

bool approx_abs(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

double parse_double(const case_block& block, const char* key) {
    auto it = block.values.find(key);
    assert(it != block.values.end());
    return std::stod(it->second);
}

std::string parse_string(const case_block& block, const char* key) {
    auto it = block.values.find(key);
    assert(it != block.values.end());
    return it->second;
}

void trim(std::string& text) {
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!text.empty() && is_space(static_cast<unsigned char>(text.front()))) text.erase(text.begin());
    while (!text.empty() && is_space(static_cast<unsigned char>(text.back()))) text.pop_back();
}

std::vector<case_block> load_case_blocks(const std::string& path) {
    std::ifstream input(path);
    assert(input.is_open());

    std::vector<case_block> blocks;
    case_block current;
    std::string line;
    while (std::getline(input, line)) {
        auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        trim(line);

        if (line.empty()) {
            if (!current.values.empty()) {
                blocks.push_back(current);
                current.values.clear();
            }
            continue;
        }

        auto pos = line.find('=');
        assert(pos != std::string::npos);
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        trim(key);
        trim(value);
        current.values[key] = value;
    }

    if (!current.values.empty()) {
        blocks.push_back(current);
    }
    return blocks;
}

double wrap_diff_deg(double lhs_deg, double rhs_deg) {
    double diff = std::fmod(lhs_deg - rhs_deg, 360.0);
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    return std::abs(diff);
}

} // namespace

int main() {
    std::string data_dir = TEST_DATA_DIR;
    std::string cases_path = data_dir + "/algorithm_io_cases.txt";
    auto blocks = load_case_blocks(cases_path);

    printf("=== Algorithm IO Tests ===\n");
    launch_pk_table_set pk_tables;

    for (const auto& block : blocks) {
        std::string type = parse_string(block, "type");

        if (type == "launch_pk") {
            for (const auto& kv : block.values) {
                if (kv.first.rfind("table", 0) == 0) {
                    std::string table_file = data_dir + "/" + kv.second;
                    std::string error;
                    bool loaded = pk_tables.load_core_file(table_file, &error);
                    assert(loaded && error.empty());
                }
            }

            launch_pk_request request;
            request.launcher_type = parse_string(block, "launcher_type");
            request.target_type = parse_string(block, "target_type");
            request.altitude_m = parse_double(block, "altitude_m");
            request.target_speed_mps = parse_double(block, "target_speed_mps");
            request.down_range_m = parse_double(block, "down_range_m");
            request.cross_range_m = parse_double(block, "cross_range_m");

            double actual = pk_tables.evaluate(request);
            double expected = parse_double(block, "expected");
            double tol = parse_double(block, "tol");
            assert(approx_abs(actual, expected, tol));
            printf("  launch_pk... OK\n");
        } else if (type == "j2_rates") {
            orbital_elements oe;
            oe.semi_major_axis_m = parse_double(block, "a_m");
            oe.eccentricity = parse_double(block, "e");
            oe.inclination_rad = parse_double(block, "i_deg") * constants::deg_to_rad;
            oe.raan_rad = parse_double(block, "raan_deg") * constants::deg_to_rad;
            oe.arg_periapsis_rad = parse_double(block, "argp_deg") * constants::deg_to_rad;
            oe.true_anomaly_rad = parse_double(block, "nu_deg") * constants::deg_to_rad;

            double dt_s = parse_double(block, "dt_s");
            auto after = propagate_j2_secular(oe, dt_s);
            double expected_raan = parse_double(block, "expected_raan_deg");
            double expected_argp = parse_double(block, "expected_argp_deg");
            double tol_deg = parse_double(block, "tol_deg");

            assert(wrap_diff_deg(after.raan_rad * constants::rad_to_deg, expected_raan) <= tol_deg);
            assert(wrap_diff_deg(after.arg_periapsis_rad * constants::rad_to_deg, expected_argp) <= tol_deg);
            printf("  j2_rates... OK\n");
        } else if (type == "hohmann") {
            double r1 = parse_double(block, "r1_m");
            double r2 = parse_double(block, "r2_m");
            auto result = hohmann_transfer(r1, r2);
            double expected = parse_double(block, "expected_total_dv_mps");
            double tol = parse_double(block, "tol_mps");
            assert(approx_abs(result.total_delta_v_mps, expected, tol));
            printf("  hohmann... OK\n");
        } else if (type == "false_target_rcs") {
            double actual_snr = parse_double(block, "actual_snr_linear");
            double reference_snr = parse_double(block, "reference_snr_linear");
            double reference_rcs = parse_double(block, "reference_rcs_m2");
            double actual = false_target_equivalent_rcs(actual_snr, reference_snr, reference_rcs);
            double expected = parse_double(block, "expected_m2");
            double tol = parse_double(block, "tol");
            assert(approx_abs(actual, expected, tol));
            printf("  false_target_rcs... OK\n");
        } else if (type == "flight_turn") {
            // 文件驱动的转弯方向样例：仅验证转弯输出方向是否与目标航向一致。
            auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -2000.0}, {180.0, 0.0, 0.0});
            state.heading_rad = parse_double(block, "heading_deg") * constants::deg_to_rad;
            state.roll_rad = parse_double(block, "roll_deg") * constants::deg_to_rad;

            coordinated_turn_target target;
            target.target_heading_rad = parse_double(block, "target_heading_deg") * constants::deg_to_rad;

            flight_control_limits limits;
            coordinated_turn_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expected_sign = parse_double(block, "expected_roll_sign");
            assert(command.valid);
            assert((expected_sign > 0.0 && command.commanded_roll_rad > 0.0) ||
                   (expected_sign < 0.0 && command.commanded_roll_rad < 0.0));
            printf("  flight_turn... OK\n");
        } else if (type == "flight_pull_up") {
            // 文件驱动的拉升样例：验证控制器在给定高度差下能输出向上的姿态和爬升意图。
            auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -parse_double(block, "altitude_m")},
                                                               {parse_double(block, "speed_mps"), 0.0, 0.0});
            state.pitch_rad = parse_double(block, "pitch_deg") * constants::deg_to_rad;

            pull_up_target target;
            target.target_altitude_m = parse_double(block, "target_altitude_m");
            target.target_climb_angle_rad = parse_double(block, "target_climb_deg") * constants::deg_to_rad;

            flight_control_limits limits;
            pull_up_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            assert(command.valid);
            assert(command.commanded_pitch_rad > 0.0);
            assert(command.commanded_vertical_speed_mps > 0.0);
            printf("  flight_pull_up... OK\n");
        } else if (type == "flight_level_hold") {
            // 文件驱动的平飞保持样例：验证控制器会朝目标高度方向给出小幅垂向修正。
            auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -parse_double(block, "altitude_m")},
                                                               {parse_double(block, "speed_mps"), 0.0, 0.0});
            state.pitch_rad = parse_double(block, "pitch_deg") * constants::deg_to_rad;

            level_hold_target target;
            target.target_altitude_m = parse_double(block, "target_altitude_m");

            flight_control_limits limits;
            level_hold_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expected_sign = parse_double(block, "expect_vertical_sign");
            assert(command.valid);
            assert((expected_sign > 0.0 && command.commanded_vertical_speed_mps > 0.0) ||
                   (expected_sign < 0.0 && command.commanded_vertical_speed_mps < 0.0));
            printf("  flight_level_hold... OK\n");
        } else if (type == "flight_waypoint") {
            // 文件驱动的航点跟踪样例：验证控制器会朝航点方向输出转向指令。
            auto state = flight_kinematic_state::from_velocity(
                {parse_double(block, "position_x_m"), parse_double(block, "position_y_m"), -parse_double(block, "altitude_m")},
                {parse_double(block, "speed_mps"), 0.0, 0.0});
            state.heading_rad = parse_double(block, "heading_deg") * constants::deg_to_rad;
            state.roll_rad = parse_double(block, "roll_deg") * constants::deg_to_rad;

            waypoint_track_target target;
            target.target_position_wcs = {parse_double(block, "target_x_m"),
                                          parse_double(block, "target_y_m"),
                                          -parse_double(block, "altitude_m")};

            flight_control_limits limits;
            waypoint_track_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expected_sign = parse_double(block, "expected_roll_sign");
            assert(command.valid);
            assert((expected_sign > 0.0 && command.commanded_roll_rad > 0.0) ||
                   (expected_sign < 0.0 && command.commanded_roll_rad < 0.0));
            printf("  flight_waypoint... OK\n");
        } else if (type == "flight_glideslope") {
            // 文件驱动的下滑道样例：验证控制器能同时给出横向对准和向下的进近修正。
            auto state = flight_kinematic_state::from_velocity(
                {parse_double(block, "position_x_m"), parse_double(block, "position_y_m"), -parse_double(block, "altitude_m")},
                {parse_double(block, "speed_mps"), 0.0, 0.0});
            state.heading_rad = parse_double(block, "heading_deg") * constants::deg_to_rad;
            state.roll_rad = parse_double(block, "roll_deg") * constants::deg_to_rad;
            state.pitch_rad = parse_double(block, "pitch_deg") * constants::deg_to_rad;

            approach_glideslope_target target;
            target.threshold_position_wcs = {parse_double(block, "threshold_x_m"),
                                             parse_double(block, "threshold_y_m"),
                                             -parse_double(block, "threshold_altitude_m")};

            flight_control_limits limits;
            approach_glideslope_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expected_roll_sign = parse_double(block, "expected_roll_sign");
            double expected_vertical_sign = parse_double(block, "expect_vertical_sign");
            assert(command.valid);
            assert((expected_roll_sign > 0.0 && command.commanded_roll_rad > 0.0) ||
                   (expected_roll_sign < 0.0 && command.commanded_roll_rad < 0.0));
            assert((expected_vertical_sign > 0.0 && command.commanded_vertical_speed_mps > 0.0) ||
                   (expected_vertical_sign < 0.0 && command.commanded_vertical_speed_mps < 0.0));
            printf("  flight_glideslope... OK\n");
        } else if (type == "flight_heading_hold") {
            // 文件驱动的定航向保持样例：验证控制器会把当前航向拉回目标航向。
            auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -parse_double(block, "altitude_m")},
                                                               {parse_double(block, "speed_mps"), 0.0, 0.0});
            state.heading_rad = parse_double(block, "heading_deg") * constants::deg_to_rad;
            state.roll_rad = parse_double(block, "roll_deg") * constants::deg_to_rad;

            heading_hold_target target;
            target.target_heading_rad = parse_double(block, "target_heading_deg") * constants::deg_to_rad;

            flight_control_limits limits;
            heading_hold_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expected_sign = parse_double(block, "expected_roll_sign");
            assert(command.valid);
            assert((expected_sign > 0.0 && command.commanded_roll_rad > 0.0) ||
                   (expected_sign < 0.0 && command.commanded_roll_rad < 0.0));
            printf("  flight_heading_hold... OK\n");
        } else if (type == "flight_flare") {
            // 文件驱动的拉平样例：验证控制器会抬头并减小下沉趋势。
            auto state = flight_kinematic_state::from_velocity({0.0, 0.0, -parse_double(block, "altitude_m")},
                                                               {parse_double(block, "speed_mps"), 0.0,
                                                                parse_double(block, "vertical_speed_down_mps")});
            state.pitch_rad = parse_double(block, "pitch_deg") * constants::deg_to_rad;

            flare_target target;

            flight_control_limits limits;
            flare_controller controller;
            auto command = controller.compute(state, target, limits, parse_double(block, "dt_s"));

            double expect_pitch_sign = parse_double(block, "expect_pitch_sign");
            double expect_vertical_improve = parse_double(block, "expect_vertical_improve");
            assert(command.valid);
            assert((expect_pitch_sign > 0.0 && command.commanded_pitch_rad > state.pitch_rad) ||
                   (expect_pitch_sign < 0.0 && command.commanded_pitch_rad < state.pitch_rad));
            if (expect_vertical_improve > 0.0) {
                assert(command.commanded_vertical_speed_mps > state.vertical_speed_mps);
            }
            printf("  flight_flare... OK\n");
        } else {
            assert(false && "unknown IO case type");
        }
    }

    printf("All algorithm IO tests passed.\n");
    return 0;
}
