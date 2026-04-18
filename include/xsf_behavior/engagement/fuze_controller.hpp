#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/vec3.hpp>
#include <xsf_math/lethality/fuze.hpp>
#include <xsf_math/lethality/pk_model.hpp>

namespace xsf_math {

// 交战行为层：引信与杀伤评估控制器。
//
// 组合 fuze.hpp 中的两阶段 PCA、近炸引信和 pk_model 中的 pk_curve/miss 降级，
// 给出“本次驻留是否应该起爆 + 预估 Pk”的意图输出。
// 行为层只做判定，不改变武器/目标状态。

struct fuze_inputs {
    double flight_time_s = 0.0;
    vec3   weapon_pos{};
    vec3   weapon_vel{};
    vec3   target_pos{};
    vec3   target_vel{};
    double track_error_increase_m = 0.0;  // 来自 EW/跟踪退化的估计
    double dt_s = 0.020;
};

struct fuze_decision {
    fuze_result status = fuze_result::no_trigger;
    bool   in_coarse_gate = false;
    bool   in_fine_gate   = false;
    double miss_distance_m = 1e10;
    double time_to_cpa_s   = 0.0;
    double effective_miss_m = 1e10;  // 叠加 EW 退化后的脱靶量
    double estimated_pk    = 0.0;
    bool   burst           = false;  // 便于调用方快速读取
};

struct fuze_controller {
    pca_two_stage  pca{};
    proximity_fuze fuze{};
    pk_curve       pk{};

    fuze_decision evaluate(const fuze_inputs& in) const {
        fuze_decision d;

        // 第 1 / 2 阶段 PCA
        auto stage = pca.check(in.weapon_pos, in.target_pos);
        d.in_coarse_gate  = stage.in_coarse_gate;
        d.in_fine_gate    = stage.in_fine_gate;
        d.miss_distance_m = stage.distance_m;

        // 引信触发判定
        d.status = fuze.check(in.flight_time_s,
                              in.weapon_pos, in.weapon_vel,
                              in.target_pos, in.target_vel,
                              in.dt_s);
        d.burst = (d.status == fuze_result::proximity_burst ||
                   d.status == fuze_result::contact);

        // CPA / time-to-CPA 填充
        auto cpa = compute_cpa(in.weapon_pos, in.weapon_vel,
                               in.target_pos, in.target_vel);
        d.time_to_cpa_s = cpa.time_to_cpa_s;

        // 叠加 EW 跟踪误差后的等效脱靶量
        d.effective_miss_m = ew_degraded_miss_distance(cpa.miss_distance_m,
                                                        in.track_error_increase_m);
        d.estimated_pk = pk.evaluate(d.effective_miss_m);

        XSF_LOG_DEBUG("fuze controller: range={:.2f}m coarse={} fine={} burst={} eff_miss={:.2f}m Pk={:.3f}",
                      stage.distance_m, d.in_coarse_gate, d.in_fine_gate,
                      d.burst, d.effective_miss_m, d.estimated_pk);
        return d;
    }
};

} // namespace xsf_math
