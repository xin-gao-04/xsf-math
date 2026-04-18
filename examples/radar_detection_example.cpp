#include <xsf_math/xsf_math.hpp>
#include <xsf_common/log.hpp>
#include <xsf_common/validation_artifacts.hpp>
#include <cstdio>
#include <cmath>

using namespace xsf_math;

int main(int argc, char** argv) {
    const auto cli = xsf::validation::parse_cli(argc, argv);
    xsf::validation::case_artifacts artifacts("radar_detection", cli);

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
    double first_subhalf_range_km = -1.0;
    double prev_snr_db = 1.0e30;
    bool snr_monotonic = true;

    if (artifacts.enabled()) {
        artifacts.write_timeseries_header({
            "range_km", "snr_db", "pd_sw0", "pd_sw1", "pd_sw3"
        });
    }

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
        snr_monotonic = snr_monotonic && (result.snr_db <= prev_snr_db + 1.0e-9);
        prev_snr_db = result.snr_db;

        // Per-range trace (high verbosity — compiled out unless level ≤ TRACE)
        XSF_LOG_TRACE("range={:.0f}km  SNR={:.1f}dB  Pd(Sw0)={:.3f}  "
                      "Pd(Sw1)={:.3f}  Pd(Sw3)={:.3f}",
                      range_km, result.snr_db, pd0, pd1, pd3);

        // Note the range where Pd first drops below 0.5
        if (first_pd_above_half && pd0 < 0.5) {
            XSF_LOG_INFO("Pd(Sw0) falls below 0.5 at range={:.0f}km  SNR={:.1f}dB",
                         range_km, result.snr_db);
            first_pd_above_half = false;
            first_subhalf_range_km = range_km;
            artifacts.append_event(range_km,
                                   "pd_drop_below_half",
                                   "Sw0 detection probability dropped below 0.5",
                                   {
                                       xsf::validation::make_field("range_km", range_km, 1),
                                       xsf::validation::make_field("snr_db", result.snr_db, 3),
                                       xsf::validation::make_field("pd_sw0", pd0, 6)
                                   });
        }

        artifacts.append_timeseries_row({
            xsf::validation::make_field("", range_km, 1).value,
            xsf::validation::make_field("", result.snr_db, 6).value,
            xsf::validation::make_field("", pd0, 6).value,
            xsf::validation::make_field("", pd1, 6).value,
            xsf::validation::make_field("", pd3, 6).value
        });

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
        artifacts.append_event(50.0,
                               "burnthrough_warning",
                               "Burnthrough range is unusually short for this configuration",
                               {
                                   xsf::validation::make_field("burnthrough_km", bt / 1000.0, 3),
                                   xsf::validation::make_field("js_db", linear_to_db(js), 3)
                               });
    }

    printf("J/S at 50km: %.1f dB\n", linear_to_db(js));
    printf("Burnthrough range: %.1f km\n", bt / 1000.0);

    const bool passed =
        snr_monotonic &&
        first_subhalf_range_km > 0.0 &&
        det_range > 0.0 &&
        std::abs(first_subhalf_range_km - det_range / 1000.0) < 10.0;
    const std::string failure_reason = passed
        ? std::string()
        : "SNR trend or detection-range crossing check failed";

    artifacts.write_metrics({
        xsf::validation::make_field("snr_monotonic", snr_monotonic),
        xsf::validation::make_field("first_subhalf_range_km", first_subhalf_range_km, 3),
        xsf::validation::make_field("detection_range_km", det_range / 1000.0, 3),
        xsf::validation::make_field("js_db_50km", linear_to_db(js), 3),
        xsf::validation::make_field("burnthrough_range_km", bt / 1000.0, 3)
    });
    artifacts.write_summary(
        passed,
        "Validate radar SNR and Pd trends across range and capture a reproducible detection crossover.",
        "SNR should decrease with range, Sw0 Pd should collapse near the computed detection range, and EW metrics should stay observable.",
        passed
            ? "Range-SNR-Pd relationship stayed monotonic and the Pd crossover remained close to the computed detection range."
            : "Observed radar trend deviated from the expected monotonic SNR decay or crossover consistency.",
        {
            xsf::validation::make_field("peak_power_kw", tx.peak_power_w / 1000.0, 1),
            xsf::validation::make_field("frequency_ghz", tx.frequency_hz / 1e9, 3),
            xsf::validation::make_field("target_rcs_m2", target_rcs, 3)
        },
        {
            xsf::validation::make_field("snr_monotonic", snr_monotonic),
            xsf::validation::make_field("first_subhalf_range_km", first_subhalf_range_km, 3),
            xsf::validation::make_field("detection_range_km", det_range / 1000.0, 3),
            xsf::validation::make_field("js_db_50km", linear_to_db(js), 3),
            xsf::validation::make_field("burnthrough_range_km", bt / 1000.0, 3)
        },
        failure_reason);

    XSF_LOG_INFO("radar detection example complete");
    return (cli.strict && !passed) ? 1 : 0;
}
