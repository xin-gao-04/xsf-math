#pragma once

#include <xsf_common/log.hpp>
#include <xsf_behavior/engagement/fuze_controller.hpp>
#include <xsf_behavior/engagement/launch_computer.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>

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
};

struct engagement_command {
    engagement_phase       phase = engagement_phase::pre_engage;
    bool                   issue_launch = false;
    launch_computer_result lc_result{};
    vec3                   guidance_accel_cmd{};
    fuze_decision          fuze_decision{};
    bool                   request_break_off = false;
};

struct engagement_controller {
    launch_computer         lc{};
    proportional_nav        pn{};
    augmented_proportional_nav apn{};
    fuze_controller         fuze{};
    accel_limiter           limiter{};

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
        if (cmd.fuze_decision.in_coarse_gate) {
            cmd.phase = engagement_phase::terminal;
            raw = apn.compute_accel(ctx.geom);
        } else {
            cmd.phase = engagement_phase::midcourse;
            raw = pn.compute_accel(ctx.geom);
        }
        cmd.guidance_accel_cmd = limiter.limit(raw);
        return cmd;
    }
};

} // namespace xsf_math
