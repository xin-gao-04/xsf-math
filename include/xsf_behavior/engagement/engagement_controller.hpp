#pragma once

#include <xsf_common/log.hpp>
#include <xsf_behavior/engagement/fuze_controller.hpp>
#include <xsf_behavior/engagement/launch_computer.hpp>
#include <xsf_math/guidance/guidance_decoupling.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>
#include <xsf_math/guidance/seeker.hpp>

namespace xsf_math {

// 交战行为层：交战状态机控制器。
//
// 组合发射计算 + 制导 + 引信，按 xsf-core 常见的交战阶段语义调度：
// pre_engage → launch → midcourse → terminal → burst → post_engage。
// 本模块只给出“当前帧意图”，不直接修改武器/目标。

enum class engagement_phase {
    pre_engage,    // 收到航迹但尚未达到发射约束
    launch,        // 发射窗口内，产生发射意图
    midcourse,     // 发射后、CPA 前，按 PN 制导
    terminal,      // 进入粗门限后，切换到 APN/精确瞄准
    burst,         // 引信触发
    post_engage    // 已越过 CPA，未来可能重燃
};

struct engagement_context {
    launch_candidate        launch_cand{};
    engagement_geometry     geom{};      // 弹目几何（midcourse/terminal 使用）
    fuze_inputs             fuze_in{};
    double                  sim_time_s = 0.0;
    bool                    weapon_launched = false;
    bool                    has_weapon_attitude = false;
    euler_angles            weapon_attitude{};
    bool                    has_seeker_measurement = false;
    seeker_measurement      seeker_observation{};
};

struct engagement_command {
    engagement_phase       phase = engagement_phase::pre_engage;
    bool                   issue_launch = false;
    launch_computer_result lc_result{};
    vec3                   guidance_accel_cmd{};
    guidance_channel_command guidance_channel_cmd{};
    fuze_decision          fuze_decision{};
    bool                   request_break_off = false;
};

struct terminal_transition_criteria {
    double coarse_gate_range_m = 0.0;
    double range_threshold_m = 1500.0;
    double time_to_go_threshold_s = 8.0;
    double los_rate_threshold_rad_s = 0.01;
    int conditions_required = 1;
};

struct engagement_controller {
    launch_computer         lc{};
    proportional_nav        pn{};
    augmented_proportional_nav apn{};
    fuze_controller         fuze{};
    accel_limiter           limiter{};
    terminal_transition_criteria terminal_logic{};

    engagement_command update(const engagement_context& ctx) const {
        engagement_command cmd;

        if (!ctx.weapon_launched) {
            cmd.lc_result = lc.evaluate(ctx.launch_cand, ctx.sim_time_s);
            if (cmd.lc_result.all_constraints_met &&
                cmd.lc_result.is_valid(launch_computer_validity::intercept_point)) {
                cmd.phase = engagement_phase::launch;
                cmd.issue_launch = true;
            } else {
                cmd.phase = engagement_phase::pre_engage;
            }
            XSF_LOG_DEBUG("engagement: pre-launch ok={} phase={}",
                          cmd.lc_result.all_constraints_met,
                          static_cast<int>(cmd.phase));
            return cmd;
        }

        // 武器已在飞。先看引信。
        cmd.fuze_decision = fuze.evaluate(ctx.fuze_in);
        if (cmd.fuze_decision.burst) {
            cmd.phase = engagement_phase::burst;
            return cmd;
        }

        // 已越过 CPA 且不再接近。
        if (ctx.geom.closing_velocity() <= 0.0 && cmd.fuze_decision.time_to_cpa_s < 0.0) {
            cmd.phase = engagement_phase::post_engage;
            cmd.request_break_off = true;
            return cmd;
        }

        // 制导：粗门限内走 APN，外侧走 PN。
        vec3 raw;
        int terminal_votes = 0;
        double coarse_gate = (terminal_logic.coarse_gate_range_m > 0.0)
            ? terminal_logic.coarse_gate_range_m
            : fuze.pca.coarse_gate_m;
        if (ctx.geom.slant_range() <= coarse_gate || cmd.fuze_decision.in_coarse_gate) ++terminal_votes;
        if (ctx.fuze_in.dt_s >= 0.0 && cmd.fuze_decision.time_to_cpa_s <= terminal_logic.time_to_go_threshold_s)
            ++terminal_votes;
        if (ctx.geom.slant_range() <= terminal_logic.range_threshold_m) ++terminal_votes;
        if (ctx.geom.los_rate().magnitude() >= terminal_logic.los_rate_threshold_rad_s) ++terminal_votes;
        if (ctx.has_seeker_measurement && (ctx.seeker_observation.locked || ctx.seeker_observation.in_fov))
            ++terminal_votes;

        if (terminal_votes >= std::max(1, terminal_logic.conditions_required)) {
            cmd.phase = engagement_phase::terminal;
            raw = apn.compute_accel(ctx.geom);
        } else {
            cmd.phase = engagement_phase::midcourse;
            raw = pn.compute_accel(ctx.geom);
        }
        cmd.guidance_accel_cmd = limiter.limit(raw);
        if (ctx.has_weapon_attitude) {
            cmd.guidance_channel_cmd = decompose_guidance_accel(cmd.guidance_accel_cmd,
                                                                ctx.weapon_attitude,
                                                                ctx.geom.weapon_vel.magnitude());
        }
        return cmd;
    }
};

} // namespace xsf_math
