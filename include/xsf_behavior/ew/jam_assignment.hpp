#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/ew/electronic_warfare.hpp>
#include <cmath>

namespace xsf_math {

// 电子战行为层：干扰方式分配控制器（Electronic warfare behavior layer: jamming assignment controller）。
//
// 给定“目标 - 雷达 - 干扰机”的几何关系与干扰机参数，决定这一次应当使用：
// - 自卫式干扰（SSJ）；
// - 或站外支援干扰（SOJ）；
// 并返回估计的 J/S 与是否突破雷达的压制门限。
// 不修改武器/传感器的实际状态，只产出“用哪种干扰”的意图。
// Given target-radar-jammer geometry and jammer parameters, decides whether to use:
// - Self-Screening Jamming (SSJ);
// - or Stand-Off Jamming (SOJ);
// Returns estimated J/S and whether radar suppression threshold is exceeded.
// Does not modify actual weapon/sensor state, only outputs jamming mode intent.

// 干扰分配模式（Jam assignment mode）
enum class jam_assignment_mode {
    off,            // 关闭（Off）
    self_screening, // 自卫式干扰（Self-screening jamming）
    stand_off       // 站外支援干扰（Stand-off jamming）
};

// 干扰分配输入（Jam assignment inputs）
struct jam_assignment_inputs {
    jammer_params jammer{};  // 干扰机参数（Jammer parameters）
    double radar_tx_power_w       = 100000.0;  // 雷达发射功率瓦（Radar transmit power in watts）
    double radar_tx_gain_linear   = 1000.0;  // 雷达发射增益线性（Radar transmit gain linear）
    double radar_rx_gain_linear   = 1000.0;  // 雷达接收增益线性（Radar receive gain linear）
    double radar_rx_gain_toward_jammer = 100.0;  // 雷达指向干扰机接收增益（Radar receive gain toward jammer）
    double radar_freq_hz          = 10.0e9;  // 雷达频率赫兹（Radar frequency in Hz）
    double radar_bandwidth_hz     = 1.0e6;  // 雷达带宽赫兹（Radar bandwidth in Hz）
    double target_rcs_m2          = 5.0;  // 目标 RCS 平方米（Target RCS in m²）
    double target_range_m         = 100000.0;  // 目标距离米（Target range in meters）
    double jammer_range_m         = 80000.0;   // 干扰机→雷达距离米（SOJ 使用）（Jammer-to-radar range in meters, for SOJ）
    double js_threshold_linear    = 1.0;       // J/S 达到此值视为有效压制（J/S threshold for effective suppression）
};

// 干扰分配指令（Jam assignment command）
struct jam_assignment_command {
    jam_assignment_mode mode = jam_assignment_mode::off;  // 干扰模式（Jamming mode）
    double js_ratio_linear   = 0.0;  // J/S 比值线性（J/S ratio linear）
    double burnthrough_range_m = 0.0;     // 仅 SSJ 有效（Burnthrough range, SSJ only）
    bool   effective         = false;     // J/S >= 门限（Effective if J/S >= threshold）
};

// 干扰分配控制器（Jam assignment controller）
struct jam_assignment_controller {
    // 简单策略：若 SOJ 的 J/S 更优，则用 SOJ；否则 SSJ（与携带式干扰机的实战经验一致）。
    // 简单策略（Simple strategy: use SOJ if its J/S is better, otherwise SSJ, consistent with self-protection jammer experience）
    jam_assignment_command select(const jam_assignment_inputs& in) const {
        jam_assignment_command cmd;

        double ssj_js = self_screening_jam::jam_to_signal(
            in.jammer.erp_w, in.jammer.bandwidth_hz,
            in.radar_tx_power_w, in.radar_tx_gain_linear, in.radar_rx_gain_linear,
            in.radar_freq_hz, in.radar_bandwidth_hz,
            in.target_rcs_m2, in.target_range_m);

        double soj_js = stand_off_jam::jam_to_signal(
            in.jammer.erp_w, in.jammer.bandwidth_hz, in.jammer_range_m,
            in.radar_tx_power_w, in.radar_tx_gain_linear,
            in.radar_rx_gain_toward_jammer,
            in.radar_freq_hz, in.radar_bandwidth_hz,
            in.target_rcs_m2, in.target_range_m);

        if (soj_js >= ssj_js && soj_js > 0.0) {
            cmd.mode = jam_assignment_mode::stand_off;
            cmd.js_ratio_linear = soj_js;
        } else if (ssj_js > 0.0) {
            cmd.mode = jam_assignment_mode::self_screening;
            cmd.js_ratio_linear = ssj_js;
            cmd.burnthrough_range_m = self_screening_jam::burnthrough_range_m(
                in.jammer.erp_w, in.jammer.bandwidth_hz,
                in.radar_tx_power_w, in.radar_tx_gain_linear,
                in.radar_freq_hz, in.radar_bandwidth_hz,
                in.target_rcs_m2);
        }
        cmd.effective = cmd.js_ratio_linear >= in.js_threshold_linear;
        XSF_LOG_DEBUG("jam assignment: mode={} J/S={:.3f} effective={}",
                      static_cast<int>(cmd.mode), cmd.js_ratio_linear, cmd.effective);
        return cmd;
    }
};

} // namespace xsf_math
