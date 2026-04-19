#pragma once

#include <xsf_common/log.hpp>
#include <xsf_behavior/engagement/fuze_controller.hpp>
#include <xsf_behavior/engagement/launch_computer.hpp>
#include <xsf_math/guidance/guidance_decoupling.hpp>
#include <xsf_math/guidance/proportional_nav.hpp>
#include <xsf_math/guidance/seeker.hpp>

namespace xsf_math {

// 交战行为层：交战状态机控制器（Engagement behavior layer: engagement state machine controller）。
//
// 组合发射计算 + 制导 + 引信，按 xsf-core 常见的交战阶段语义调度：
// pre_engage → launch → midcourse → terminal → burst → post_engage。
// 本模块只给出“当前帧意图”，不直接修改武器/目标。
// Combines launch computation, guidance, and fuze, scheduling by xsf-core engagement phase semantics.
// This module only outputs "current frame intent" without directly modifying weapon/target state.

enum class engagement_phase {
    pre_engage,    // 收到航迹但尚未达到发射约束（Track received but launch constraints not yet met）
    launch,        // 发射窗口内，产生发射意图（Within launch window, generate launch intent）
    midcourse,     // 发射后、CPA 前，按 PN 制导（Post-launch, pre-CPA, PN guidance）
    terminal,      // 进入粗门限后，切换到 APN/精确瞄准（Entered coarse gate, switch to APN/precision aim）
    burst,         // 引信触发（Fuze trigger）
    post_engage    // 已越过 CPA，未来可能重燃（Passed CPA, possible re-engagement later）
};

struct engagement_context {
    launch_candidate        launch_cand{};  // 发射候选（Launch candidate）
    engagement_geometry     geom{};      // 弹目几何（midcourse/terminal 使用）（Engagement geometry, used in midcourse/terminal）
    fuze_inputs             fuze_in{};  // 引信输入（Fuze inputs）
    double                  sim_time_s = 0.0;  // 仿真时间秒（Simulation time in seconds）
    bool                    weapon_launched = false;  // 武器是否已发射（Weapon launched flag）
    bool                    has_weapon_attitude = false;  // 是否有武器姿态（Has weapon attitude）
    euler_angles            weapon_attitude{};  // 武器姿态（Weapon attitude）
    bool                    has_seeker_measurement = false;  // 是否有导引头量测（Has seeker measurement）
    seeker_measurement      seeker_observation{};  // 导引头观测（Seeker observation）
};

struct engagement_command {
    engagement_phase       phase = engagement_phase::pre_engage;  // 当前交战阶段（Current engagement phase）
    bool                   issue_launch = false;  // 是否发出发射指令（Issue launch command）
    launch_computer_result lc_result{};  // 发射计算结果（Launch computer result）
    vec3                   guidance_accel_cmd{};  // 制导加速度指令（Guidance acceleration command）
    guidance_channel_command guidance_channel_cmd{};  // 制导通道指令（Guidance channel command）
    fuze_decision          fuze_decision{};  // 引信决策（Fuze decision）
    bool                   request_break_off = false;  // 请求脱离（Request break-off）
};

struct terminal_transition_criteria {
    double coarse_gate_range_m = 0.0;  // 粗门限距离米（Coarse gate range in meters）
    double range_threshold_m = 1500.0;  // 距离门限米（Range threshold in meters）
    double time_to_go_threshold_s = 8.0;  // 剩余时间门限秒（Time-to-go threshold in seconds）
    double los_rate_threshold_rad_s = 0.01;  // 视线角速度门限 rad/s（LOS rate threshold in rad/s）
    int conditions_required = 1;  // 所需满足条件数（Conditions required）
};

struct engagement_controller {
    launch_computer         lc{};  // 发射计算器（Launch computer）
    proportional_nav        pn{};  // 比例导引（Proportional navigation）
    augmented_proportional_nav apn{};  // 增广比例导引（Augmented proportional navigation）
    fuze_controller         fuze{};  // 引信控制器（Fuze controller）
    accel_limiter           limiter{};  // 加速度限制器（Acceleration limiter）
    terminal_transition_criteria terminal_logic{};  // 末端过渡判据（Terminal transition criteria）

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

        // 武器已在飞。先看引信。（Weapon in flight. Check fuze first.）
        cmd.fuze_decision = fuze.evaluate(ctx.fuze_in);
        if (cmd.fuze_decision.burst) {
            cmd.phase = engagement_phase::burst;
            return cmd;
        }

        // 已越过 CPA 且不再接近。（Passed CPA and no longer closing.）
        if (ctx.geom.closing_velocity() <= 0.0 && cmd.fuze_decision.time_to_cpa_s < 0.0) {
            cmd.phase = engagement_phase::post_engage;
            cmd.request_break_off = true;
            return cmd;
        }

        // 制导：粗门限内走 APN，外侧走 PN。（Guidance: APN inside coarse gate, PN outside.）
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
