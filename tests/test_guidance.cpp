#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace xsf_math;

static bool approx_rel(double a, double b, double tol = 0.1) {
    if (std::abs(b) < 1e-20) return std::abs(a) < tol;
    return std::abs((a - b) / b) < tol;
}

void test_engagement_geometry() {
    printf("  engagement_geometry... ");
    engagement_geometry g;
    g.weapon_pos = {0, 0, 0};
    g.weapon_vel = {300, 0, 0};
    g.target_pos = {10000, 0, 0};
    g.target_vel = {-300, 0, 0};

    assert(approx_rel(g.slant_range(), 10000.0, 0.001));
    assert(approx_rel(g.closing_velocity(), 600.0, 0.01));
    assert(approx_rel(g.time_to_intercept(), 10000.0/600.0, 0.01));

    // 正面接近时 LOS 角速度应接近 0
    vec3 omega = g.los_rate();
    assert(omega.magnitude() < 0.01);

    printf("OK\n");
}

void test_proportional_nav() {
    printf("  proportional_nav... ");

    // 正面交战：没有 LOS 角速度 -> 不应有导引修正
    proportional_nav pn;
    pn.nav_ratio = 4.0;

    engagement_geometry g;
    g.weapon_pos = {0, 0, 0};
    g.weapon_vel = {300, 0, 0};
    g.target_pos = {10000, 0, 0};
    g.target_vel = {-300, 0, 0};

    vec3 a = pn.compute_accel(g);
    assert(a.magnitude() < 1.0);  // 正面接近时应接近 0

    // 横向穿越目标：应产生侧向加速度
    g.target_pos = {10000, 100, 0};
    g.target_vel = {0, 300, 0};  // 垂直方向运动
    a = pn.compute_accel(g);
    assert(a.magnitude() > 0.1);  // 应产生修正

    printf("OK\n");
}

void test_augmented_pn() {
    printf("  augmented_pn... ");

    augmented_proportional_nav apn;
    apn.nav_ratio = 4.0;

    engagement_geometry g;
    g.weapon_pos = {0, 0, 0};
    g.weapon_vel = {500, 0, 0};
    g.target_pos = {5000, 500, 0};
    g.target_vel = {0, 200, 0};
    g.target_accel = {0, 50, 0};  // 目标机动

    vec3 a_apn = apn.compute_accel(g);

    // 与基础 PN 对比
    proportional_nav pn;
    pn.nav_ratio = 4.0;
    vec3 a_pn = pn.compute_accel(g);

    // 由于目标加速度项，APN 应产生更大的修正
    assert(a_apn.magnitude() > a_pn.magnitude());

    printf("OK\n");
}

void test_pursuit_guidance() {
    printf("  pursuit_guidance... ");

    pursuit_guidance pursuit;
    pursuit.gain = 3.0;

    engagement_geometry g;
    g.weapon_pos = {0, 0, 0};
    g.weapon_vel = {300, 0, 0};
    g.target_pos = {10000, 5000, 0};  // 目标在侧方
    g.target_vel = {0, 0, 0};

    vec3 a = pursuit.compute_accel(g);
    assert(a.magnitude() > 0.1);  // 应朝向目标转向
    assert(a.y > 0);  // 应向右转（朝向正 Y）

    printf("OK\n");
}

void test_accel_limiter() {
    printf("  accel_limiter... ");

    accel_limiter lim;
    lim.max_g = 10.0;

    vec3 cmd = {0, 0, -200.0};  // 约 20g 指令
    vec3 limited = lim.limit(cmd);
    double g_load = limited.magnitude() / constants::gravity_mps2;
    assert(approx_rel(g_load, 10.0, 0.01));

    // 小指令应原样通过
    vec3 small_cmd = {0, 5.0, 0};
    vec3 small_limited = lim.limit(small_cmd);
    assert(approx_rel(small_limited.y, 5.0, 0.001));

    printf("OK\n");
}

void test_fuze() {
    printf("  fuze... ");

    // CPA 计算
    vec3 wp = {0, 0, 0}, wv = {100, 0, 0};
    vec3 tp = {1000, 50, 0}, tv = {-100, 0, 0};
    auto cpa = compute_cpa(wp, wv, tp, tv);
    assert(cpa.time_to_cpa_s > 0);
    assert(cpa.miss_distance_m > 0 && cpa.miss_distance_m < 100);

    // Pk 曲线
    auto pk = pk_curve::blast_fragmentation(10.0, 30.0);
    double pk0 = pk.evaluate(0.0);
    assert(pk0 >= 0.99);  // 直接命中 -> 高 Pk

    double pk_far = pk.evaluate(30.0);
    assert(pk_far < 0.1);  // 距离较远 -> 低 Pk

    printf("OK\n");
}

void test_ew() {
    printf("  electronic_warfare... ");

    // 自卫式压制干扰
    double js = self_screening_jam::jam_to_signal(
        1000.0, 1e9,   // 1kW 干扰机，1GHz 带宽
        100000.0,       // 100kW 雷达
        1000.0,         // Gt = 30 dBi
        1000.0,         // Gr = 30 dBi
        10e9, 1e6,      // 10 GHz，1 MHz 带宽
        1.0,            // 1 m^2 RCS
        50000.0);       // 50 km 距离
    assert(js > 0);

    // 应存在穿透距离
    double bt = self_screening_jam::burnthrough_range_m(
        1000.0, 1e9,
        100000.0, 1000.0,
        10e9, 1e6, 1.0);
    assert(bt > 0 && bt < 200000.0);

    printf("OK\n");
}

void test_aero() {
    printf("  aerodynamics... ");

    aero_2d aero;
    aero.ref_area_m2 = 30.0;
    aero.aspect_ratio = 8.0;
    aero.oswald_factor = 0.8;
    aero.cd0 = 0.02;
    aero.cl_max = 1.5;

    auto state = aero_state::from_alt_speed(5000, 250.0);
    assert(state.dynamic_pressure_pa > 0);
    assert(state.mach > 0);

    // 阻力应为正
    auto forces = aero.compute(state, 1.0);
    assert(forces.drag_n > 0);

    // 最大过载
    double max_g = aero.max_g_available(state, 10000.0);
    assert(max_g > 0);

    printf("OK\n");
}

void test_orbital() {
    printf("  orbital... ");

    // 圆轨道
    orbital_elements oe;
    oe.semi_major_axis_m = constants::earth_radius_m + 400000.0;
    oe.eccentricity = 0.0;
    oe.inclination_rad = 0.0;
    oe.true_anomaly_rad = 0.0;

    // 400km 轨道周期应约为 92 分钟
    double period_min = oe.period() / 60.0;
    assert(period_min > 90 && period_min < 95);

    // 开普勒求解器
    double E = solve_kepler(1.0, 0.5);
    double M_check = E - 0.5 * std::sin(E);
    assert(approx_rel(M_check, 1.0, 0.0001));

    // 推进：一个周期后应回到相同位置
    auto after = propagate_kepler(oe, oe.period());
    assert(std::abs(after.true_anomaly_rad) < 0.01 ||
           std::abs(after.true_anomaly_rad - constants::two_pi) < 0.01);

    printf("OK\n");
}

int main() {
    printf("=== Guidance/Aero/EW/Lethality Tests ===\n");
    test_engagement_geometry();
    test_proportional_nav();
    test_augmented_pn();
    test_pursuit_guidance();
    test_accel_limiter();
    test_fuze();
    test_ew();
    test_aero();
    test_orbital();
    printf("All guidance/aero/EW/lethality tests passed.\n");
    return 0;
}
