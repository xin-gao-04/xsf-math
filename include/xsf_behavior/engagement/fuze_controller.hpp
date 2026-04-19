#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/vec3.hpp>
#include <xsf_math/lethality/fuze.hpp>
#include <xsf_math/lethality/pk_model.hpp>

namespace xsf_math {

// 交战行为层：引信与杀伤评估控制器（Engagement behavior layer: fuze and lethality assessment controller）。
//
// 组合 fuze.hpp 中的两阶段 PCA、近炸引信和 pk_model 中的 pk_curve/miss 降级，
// 给出“本次驻留是否应该起爆 + 预估 Pk”的意图输出。
// 行为层只做判定，不改变武器/目标状态。
// Combines two-stage PCA from fuze.hpp, proximity fuze, and pk_curve/miss degradation from pk_model,
// outputting intent "should burst this dwell + estimated Pk".
// Behavior layer only decides, does not modify weapon/target state.

struct fuze_inputs {
    double flight_time_s = 0.0;  // 飞行时间秒（Flight time in seconds）
    vec3   weapon_pos{};  // 武器位置（Weapon position）
    vec3   weapon_vel{};  // 武器速度（Weapon velocity）
    vec3   target_pos{};  // 目标位置（Target position）
    vec3   target_vel{};  // 目标速度（Target velocity）
    double track_error_increase_m = 0.0;  // 来自 EW/跟踪退化的估计（Estimated track error increase from EW/tracking degradation）
    double dt_s = 0.020;  // 时间步长秒（Time step in seconds）
};

struct fuze_decision {
    fuze_result status = fuze_result::no_trigger;  // 引信状态（Fuze status）
    bool   in_coarse_gate = false;  // 是否在粗门限内（In coarse gate）
    bool   in_fine_gate   = false;  // 是否在细门限内（In fine gate）
    double miss_distance_m = 1e10;  // 脱靶距离米（Miss distance in meters）
    double time_to_cpa_s   = 0.0;  // 到最近接近点时间秒（Time to closest point of approach in seconds）
    double effective_miss_m = 1e10;  // 叠加 EW 退化后的脱靶量（Effective miss after EW degradation）
    double estimated_pk    = 0.0;  // 预估杀伤概率（Estimated kill probability）
    bool   burst           = false;  // 便于调用方快速读取（Burst flag for quick access）
};

struct fuze_controller {
    pca_two_stage  pca{};  // 两阶段近距接近分析（Two-stage proximity close approach）
    proximity_fuze fuze{};  // 近炸引信（Proximity fuze）
    pk_curve       pk{};  // 杀伤概率曲线（Kill probability curve）

    fuze_decision evaluate(const fuze_inputs& in) const {
        fuze_decision d;

        // 第 1 / 2 阶段 PCA（Stage 1/2 PCA）
        auto stage = pca.check(in.weapon_pos, in.target_pos);
        d.in_coarse_gate  = stage.in_coarse_gate;
        d.in_fine_gate    = stage.in_fine_gate;
        d.miss_distance_m = stage.distance_m;

        // 引信触发判定（Fuze trigger decision）
        d.status = fuze.check(in.flight_time_s,
                              in.weapon_pos, in.weapon_vel,
                              in.target_pos, in.target_vel,
                              in.dt_s);
        d.burst = (d.status == fuze_result::proximity_burst ||
                   d.status == fuze_result::contact);

        // CPA / time-to-CPA 填充（CPA / time-to-CPA computation）
        auto cpa = compute_cpa(in.weapon_pos, in.weapon_vel,
                               in.target_pos, in.target_vel);
        d.time_to_cpa_s = cpa.time_to_cpa_s;

        // 叠加 EW 跟踪误差后的等效脱靶量（Effective miss distance after EW tracking error degradation）
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
