#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/ew/electronic_warfare.hpp>
#include <cmath>

namespace xsf_math {

// 电子战行为层：干扰方式分配控制器。
//
// 给定“目标 - 雷达 - 干扰机”的几何关系与干扰机参数，决定这一次应当使用：
// - 自卫式干扰（SSJ）；
// - 或站外支援干扰（SOJ）；
// 并返回估计的 J/S 与是否突破雷达的压制门限。
// 不修改武器/传感器的实际状态，只产出“用哪种干扰”的意图。

enum class jam_assignment_mode {
    off,
    self_screening,
    stand_off
};

struct jam_assignment_inputs {
    jammer_params jammer{};
    double radar_tx_power_w       = 100000.0;
    double radar_tx_gain_linear   = 1000.0;
    double radar_rx_gain_linear   = 1000.0;
    double radar_rx_gain_toward_jammer = 100.0;
    double radar_freq_hz          = 10.0e9;
    double radar_bandwidth_hz     = 1.0e6;
    double target_rcs_m2          = 5.0;
    double target_range_m         = 100000.0;
    double jammer_range_m         = 80000.0;   // 干扰机→雷达（SOJ 使用）
    double js_threshold_linear    = 1.0;       // J/S 达到此值视为有效压制
};

struct jam_assignment_command {
    jam_assignment_mode mode = jam_assignment_mode::off;
    double js_ratio_linear   = 0.0;
    double burnthrough_range_m = 0.0;     // 仅 SSJ 有效
    bool   effective         = false;     // J/S >= 门限
};

struct jam_assignment_controller {
    // 简单策略：若 SOJ 的 J/S 更优，则用 SOJ；否则 SSJ（与携带式干扰机的实战经验一致）。
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
