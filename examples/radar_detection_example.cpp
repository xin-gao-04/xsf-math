#include <xsf_math/xsf_math.hpp>
#include <xsf_common/log.hpp>
#include <cstdio>

using namespace xsf_math;

int main() {
    printf("=== Radar Detection Chain Example ===\n\n");

    // 1. 配置雷达
    transmitter_params tx;
    tx.peak_power_w   = 100000.0;   // 100 kW
    tx.frequency_hz   = 10.0e9;     // X 波段（10 GHz）
    tx.bandwidth_hz   = 1.0e6;
    tx.system_loss_db = 4.0;

    receiver_params rx;
    rx.noise_figure_db = 3.0;
    rx.noise_temp_k    = 290.0;
    rx.bandwidth_hz    = 1.0e6;

    XSF_LOG_INFO("radar: Pt={:.0f}kW  f={:.1f}GHz  NF={:.1f}dB  Bn={:.0f}kHz",
                 tx.peak_power_w / 1000.0, tx.frequency_hz / 1e9,
                 rx.noise_figure_db, rx.bandwidth_hz / 1000.0);

    printf("Radar: Pt=%.0f kW, f=%.1f GHz, NF=%.1f dB\n",
           tx.peak_power_w / 1000.0, tx.frequency_hz / 1e9, rx.noise_figure_db);
    printf("Noise power: %.2e W\n\n", rx.noise_power_w());

    // 2. 天线
    antenna_cosine ant;
    ant.peak_gain_db = 35.0;
    ant.half_bw_az_rad = 2.0 * constants::deg_to_rad;
    ant.half_bw_el_rad = 2.0 * constants::deg_to_rad;

    // 3. 扫描不同距离
    printf("Range(km)  SNR(dB)   Pd(Sw0)   Pd(Sw1)   Pd(Sw3)\n");
    printf("--------  --------  --------  --------  --------\n");

    marcum_swerling det_sw0, det_sw1, det_sw3;
    det_sw0.swerling_case = 0;
    det_sw0.num_pulses_integrated = 10;
    det_sw1.swerling_case = 1;
    det_sw1.num_pulses_integrated = 10;
    det_sw3.swerling_case = 3;
    det_sw3.num_pulses_integrated = 10;

    double target_rcs = 1.0;  // 1 m^2（0 dBsm）
    bool first_pd_above_half = true;

    for (double range_km = 10; range_km <= 200; range_km += 10) {
        radar_geometry geom;
        geom.range_m = range_km * 1000.0;
        geom.target_rcs_m2 = target_rcs;
        geom.tx_antenna_gain_db = ant.peak_gain_db;
        geom.rx_antenna_gain_db = ant.peak_gain_db;

        auto result = monostatic_radar_equation(tx, rx, geom);
        double pd0 = det_sw0.compute_pd(result.snr_linear);
        double pd1 = det_sw1.compute_pd(result.snr_linear);
        double pd3 = det_sw3.compute_pd(result.snr_linear);

        // Per-range trace (high verbosity — compiled out unless level ≤ TRACE)
        XSF_LOG_TRACE("range={:.0f}km  SNR={:.1f}dB  Pd(Sw0)={:.3f}  "
                      "Pd(Sw1)={:.3f}  Pd(Sw3)={:.3f}",
                      range_km, result.snr_db, pd0, pd1, pd3);

        // Note the range where Pd first drops below 0.5
        if (first_pd_above_half && pd0 < 0.5) {
            XSF_LOG_INFO("Pd(Sw0) falls below 0.5 at range={:.0f}km  SNR={:.1f}dB",
                         range_km, result.snr_db);
            first_pd_above_half = false;
        }

        printf("%6.0f    %8.1f  %8.3f  %8.3f  %8.3f\n",
               range_km, result.snr_db, pd0, pd1, pd3);
    }

    // 4. 探测距离
    double det_range = compute_detection_range(tx, rx, target_rcs, ant.peak_gain_db, det_sw0);
    XSF_LOG_INFO("max detection range (Sw0 Pd=0.5): {:.1f}km", det_range / 1000.0);
    printf("\nMax detection range (Sw0, Pd=0.5): %.1f km\n", det_range / 1000.0);

    // 5. 电子战效应
    printf("\n=== Electronic Warfare Effect ===\n");
    double js = self_screening_jam::jam_to_signal(
        500.0, 1e9,  // 500W 干扰机 ERP，1GHz 带宽
        tx.peak_power_w, db_to_linear(ant.peak_gain_db), db_to_linear(ant.peak_gain_db),
        tx.frequency_hz, rx.bandwidth_hz, target_rcs, 50000.0);
    double bt = self_screening_jam::burnthrough_range_m(
        500.0, 1e9,
        tx.peak_power_w, db_to_linear(ant.peak_gain_db),
        tx.frequency_hz, rx.bandwidth_hz, target_rcs);

    XSF_LOG_INFO("EW: J/S at 50km = {:.1f}dB  burnthrough = {:.1f}km",
                 linear_to_db(js), bt / 1000.0);

    if (bt < 1000.0) {
        XSF_LOG_WARN("burnthrough range {:.0f}m is unusually short — "
                     "check jammer ERP and radar parameters", bt);
    }

    printf("J/S at 50km: %.1f dB\n", linear_to_db(js));
    printf("Burnthrough range: %.1f km\n", bt / 1000.0);

    XSF_LOG_INFO("radar detection example complete");
    return 0;
}
