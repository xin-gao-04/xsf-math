#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <xsf_math/core/vec3.hpp>
#include <xsf_math/radar/radar_equation.hpp>
#include <xsf_math/radar/marcum_swerling.hpp>
#include <xsf_math/ew/electronic_warfare.hpp>
#include <random>

namespace xsf_math {

// 感知行为层：探测决策控制器（Sensor behavior layer: detection decision controller）。
//
// 职责：把雷达方程 + 统计检测 + EW 降级组合成一次“本次驻留是否判为有目标”的意图输出。
// 上层给出几何/RCS/干扰态势，本控制器完成：
//   1. 计算名义 SNR；
//   2. 叠加杂波 / 干扰 / 噪声乘子；
//   3. 经 marcum_swerling 映射到 Pd；
//   4. 按伯努利抽样产出本次驻留的检测决策（supports deterministic Pd-threshold mode）。
// 不维护航迹或调度状态；那两件事分别由 tracking 和 sensor_scheduler 处理。
// Responsibility: combine radar equation + statistical detection + EW degradation into
// a single intent output "does this dwell detect a target".
// Upper layer provides geometry/RCS/jamming situation; this controller:
//   1. Computes nominal SNR;
//   2. Applies clutter / jamming / noise multipliers;
//   3. Maps to Pd via marcum_swerling;
//   4. Produces detection decision by Bernoulli sampling (supports deterministic Pd-threshold mode).
// Does not maintain tracks or scheduling state; those are handled by tracking and sensor_scheduler.

// 探测输入（Detection inputs）
struct detection_inputs {
    radar_geometry        geometry{};      // 含 range / RCS / 天线增益 / 俯仰（Includes range, RCS, antenna gain, elevation）
    double                clutter_power_w = 0.0;  // 杂波功率瓦（Clutter power in watts）
    double                jamming_power_w = 0.0;  // 干扰功率瓦（Jamming power in watts）
    ew_degradation        ew_effect{};     // 注入式 EW 乘子（Injected EW multiplier）
    double                js_ratio_linear = 0.0;  // 若仅想用 ew_effect.degrade_snr，则给出 J/S（J/S ratio if only using ew_effect.degrade_snr）
};

// 探测决策（Detection decision）
struct detection_decision {
    bool   detected        = false;  // 是否探测到（Detected）
    double snr_linear      = 0.0;  // 线性 SNR（SNR linear）
    double snr_db          = -999.0;  // SNR 分贝（SNR in dB）
    double pd              = 0.0;  // 探测概率（Probability of detection）
    double false_alarm     = 0.0;  // 虚警概率（False alarm probability）
    double sample_draw     = 0.0;  // 伯努利抽样的随机值（Bernoulli sample draw）
};

// 探测决策控制器（Detection decision controller）
struct detection_controller {
    transmitter_params tx{};  // 发射机参数（Transmitter parameters）
    receiver_params    rx{};  // 接收机参数（Receiver parameters）
    marcum_swerling    detector{};  // 检测器（Detector）
    double             detection_threshold_linear = 1.9953;  // 约 3dB，匹配 marcum_swerling::compute_pd（Approx 3dB, matching marcum_swerling::compute_pd）
    std::mt19937       rng{42};  // 随机数生成器（Random number generator）

    // 不开随机抽样时直接按 pd_threshold 判决，方便测试。（Deterministic Pd-threshold mode for testing, no random sampling）
    bool   stochastic = true;  // 是否使用随机抽样（Use stochastic sampling）
    double pd_threshold = 0.5;  // Pd 判决门限（Pd decision threshold）

    // 单次驻留的探测决策（Single-dwell detection decision）
    detection_decision evaluate(const detection_inputs& in) {
        // 1) 基线 SNR
        auto base = monostatic_radar_equation(tx, rx, in.geometry);

        // 2) 把杂波 / 干扰 / 噪声并联进分母
        snr_with_interference mix;
        mix.signal_power_w  = base.signal_power_w;
        mix.clutter_power_w = in.clutter_power_w;
        mix.jamming_power_w = in.jamming_power_w;
        mix.noise_power_w   = base.noise_power_w;
        double snr_lin = mix.snr_linear();

        // 3) 叠加 EW 乘子（js_ratio_linear 让“间接压制”也可体现）
        snr_lin = in.ew_effect.degrade_snr(snr_lin, in.js_ratio_linear);

        // 4) Pd 与决策
        double pd = detector.compute_pd(snr_lin, detection_threshold_linear);
        detection_decision d;
        d.snr_linear  = snr_lin;
        d.snr_db      = (snr_lin > 0.0) ? linear_to_db(snr_lin) : -999.0;
        d.pd          = pd;
        d.false_alarm = detector.prob_false_alarm;

        if (stochastic) {
            std::uniform_real_distribution<double> u(0.0, 1.0);
            d.sample_draw = u(rng);
            d.detected = d.sample_draw < pd;
        } else {
            d.sample_draw = pd_threshold;
            d.detected = pd >= pd_threshold;
        }
        XSF_LOG_DEBUG("detection decision: R={:.0f}m SNR={:.2f}dB Pd={:.3f} hit={}",
                      in.geometry.range_m, d.snr_db, d.pd, d.detected);
        return d;
    }
};

} // namespace xsf_math
