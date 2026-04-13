#include <xsf_math/xsf_math.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace xsf_math;

static bool approx_rel(double a, double b, double tol = 0.05) {
    if (std::abs(b) < 1e-20) return std::abs(a) < tol;
    return std::abs((a - b) / b) < tol;
}

void test_radar_equation() {
    printf("  radar_equation... ");
    transmitter_params tx;
    tx.peak_power_w = 100000.0;
    tx.frequency_hz = 10.0e9;
    tx.system_loss_db = 0.0;

    receiver_params rx;
    rx.noise_figure_db = 0.0;
    rx.noise_temp_k = 290.0;
    rx.bandwidth_hz = 1e6;

    radar_geometry geom;
    geom.range_m = 100000.0;
    geom.target_rcs_m2 = 1.0;
    geom.tx_antenna_gain_db = 30.0;
    geom.rx_antenna_gain_db = 30.0;

    auto result = monostatic_radar_equation(tx, rx, geom);

    // 对这些参数来说，SNR 应该处于合理范围
    assert(result.snr_db > -20.0 && result.snr_db < 40.0);
    assert(result.signal_power_w > 0);
    assert(result.noise_power_w > 0);
    assert(result.max_range_m > 0);

    // 距离翻倍应使 SNR 下降约 12 dB（R^4）
    geom.range_m = 200000.0;
    auto result2 = monostatic_radar_equation(tx, rx, geom);
    double snr_drop = result.snr_db - result2.snr_db;
    assert(approx_rel(snr_drop, 12.0, 0.1));

    printf("OK\n");
}

void test_marcum_swerling() {
    printf("  marcum_swerling... ");

    marcum_swerling det;
    det.swerling_case = 0;
    det.num_pulses_integrated = 1;
    det.prob_false_alarm = 1e-6;

    // 高 SNR -> Pd 接近 1
    double pd_high = det.compute_pd(1000.0);
    assert(pd_high > 0.99);

    // 低 SNR -> Pd 接近 0
    double pd_low = det.compute_pd(0.001);
    assert(pd_low < 0.01);

    // 所需 SNR 应能得到指定的 Pd
    double req_snr = det.required_snr(0.5);
    double pd_check = det.compute_pd(req_snr);
    assert(approx_rel(pd_check, 0.5, 0.05));

    // 更多脉冲应降低单脉冲所需 SNR
    marcum_swerling det10 = det;
    det10.num_pulses_integrated = 10;
    double req_snr10 = det10.required_snr(0.5);
    assert(req_snr10 < req_snr);  // 积分有帮助

    printf("OK\n");
}

void test_antenna() {
    printf("  antenna... ");

    antenna_cosine ant;
    ant.peak_gain_db = 30.0;
    ant.half_bw_az_rad = 3.0 * constants::deg_to_rad;

    // 波束轴向的峰值增益
    auto g0 = ant.evaluate(0, 0);
    assert(approx_rel(g0.gain_db, 30.0, 0.01));
    assert(g0.in_fov);

    // 3dB 波束宽度：在 half_bw 处增益应约为 27 dB
    auto g3 = ant.evaluate(ant.half_bw_az_rad, 0);
    assert(g3.gain_db < 30.0);
    assert(g3.gain_db > 25.0);

    // 视场外
    auto g_out = ant.evaluate(70 * constants::deg_to_rad, 0);
    assert(!g_out.in_fov);
    assert(g_out.gain_db < ant.peak_gain_db - 30.0);

    printf("OK\n");
}

void test_propagation() {
    printf("  propagation... ");

    // 自由空间路径损耗会随距离和频率增加
    double fspl_10 = free_space_loss_db(10000.0, 10e9);
    double fspl_20 = free_space_loss_db(20000.0, 10e9);
    assert(fspl_20 > fspl_10);
    // 距离翻倍增加约 6 dB
    assert(approx_rel(fspl_20 - fspl_10, 6.0, 0.1));

    // 大气衰减会随距离增加
    double atm_50 = blake_attenuation::total_loss_db(50000, 10e9, 0.1);
    double atm_100 = blake_attenuation::total_loss_db(100000, 10e9, 0.1);
    assert(atm_100 > atm_50);

    printf("OK\n");
}

void test_clutter() {
    printf("  clutter... ");
    constant_gamma_clutter clut;
    clut.gamma_db = -20.0;

    // Sigma0 会随掠射角增大
    double s0_low = clut.sigma0(5.0 * constants::deg_to_rad);
    double s0_high = clut.sigma0(30.0 * constants::deg_to_rad);
    assert(s0_high > s0_low);

    // MTI 改善应为正值
    double mti = mti_improvement_factor_db(2, 5.0, 1000.0, 10e9);
    assert(mti > 0.0);

    printf("OK\n");
}

void test_rcs() {
    printf("  rcs... ");
    rcs_constant rcs;
    rcs.sigma_m2 = 10.0;
    auto val = rcs.evaluate(0, 0, 10e9);
    assert(approx_rel(val.m2, 10.0, 0.001));
    assert(approx_rel(val.dbsm, 10.0, 0.01));

    // 1 m^2 = 0 dBsm
    auto v2 = rcs_value::from_m2(1.0);
    assert(std::abs(v2.dbsm) < 0.01);

    printf("OK\n");
}

int main() {
    printf("=== Radar Module Tests ===\n");
    test_radar_equation();
    test_marcum_swerling();
    test_antenna();
    test_propagation();
    test_clutter();
    test_rcs();
    printf("All radar tests passed.\n");
    return 0;
}
