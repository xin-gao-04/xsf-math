// 串联示例：感知 → 跟踪 → 交战
//
// 场景：一架雷达站（原点）盯着一个以 250 m/s 朝自己飞来的目标。
//       每个 dt 循环一次行为层链路：
//       1. sensor_scheduler 选下一次照射；
//       2. detection_controller 判定本次驻留是否发现目标；
//       3. track_manager 按发现结果维护航迹；
//       4. 航迹确认后，engagement_controller 评估发射窗口并输出意图；
//       5. 一旦“发射”，示例里用恒速弹做简单运动，交给 fuze_controller 判定起爆。
//
// 行为层不真正推进实体，下面例子的运动学是示例代码自己算的。

#include <xsf_math/xsf_math.hpp>
#include <xsf_common/log.hpp>
#include <cstdio>

using namespace xsf_math;

namespace {

// 根据 detection_inputs 里的几何把 radar_equation 所需字段填齐。
detection_inputs make_detection_inputs(const vec3& target_pos) {
    detection_inputs in;
    in.geometry.range_m             = target_pos.magnitude();
    in.geometry.target_rcs_m2       = 5.0;
    in.geometry.tx_antenna_gain_db  = 35.0;
    in.geometry.rx_antenna_gain_db  = 35.0;
    in.clutter_power_w = 0.0;
    in.jamming_power_w = 0.0;
    return in;
}

detection make_detection(const vec3& truth_pos, std::mt19937& rng) {
    // 加一点点位置噪声，让 KF + 关联器更有活干。
    std::normal_distribution<double> noise(0.0, 20.0);
    detection d;
    d.position = {truth_pos.x + noise(rng),
                  truth_pos.y + noise(rng),
                  truth_pos.z + noise(rng)};
    d.meas_noise[0] = d.meas_noise[1] = d.meas_noise[2] = 400.0;  // 20m sigma^2
    return d;
}

}  // namespace

int main() {
    printf("=== Behavior Chain Example (sensor -> tracking -> engagement) ===\n\n");

    // --- 目标真值 ---
    vec3 target_pos = {80000.0, 0.0, -5000.0};   // 80km 外、5km 高（WCS z 向下为正）
    vec3 target_vel = {-250.0,  0.0,   0.0};     // 250 m/s 朝雷达飞来

    // --- 感知：调度器 + 探测控制器 ---
    sensor_scheduler scheduler;
    scheduler.params.search_frame_time_s = 0.5;
    scheduler.params.dwell_time_s        = 0.020;
    scheduler.params.track_revisit_time_s = 0.25;
    scheduler.search_list.push_back({0});        // 唯一目标

    detection_controller sensor;
    sensor.tx.peak_power_w   = 200000.0;
    sensor.tx.frequency_hz   = 10.0e9;
    sensor.tx.bandwidth_hz   = 1.0e6;
    sensor.rx.noise_figure_db = 3.0;
    sensor.rx.bandwidth_hz   = 1.0e6;
    sensor.detector.swerling_case = 1;
    sensor.detector.num_pulses_integrated = 16;
    sensor.stochastic = false;          // 用 Pd 门限比较，便于复现
    sensor.pd_threshold = 0.5;

    // --- 跟踪：航迹管理器 ---
    track_manager tracker;
    tracker.params.mofn.m = 3;
    tracker.params.mofn.n = 5;
    tracker.params.drop_after_misses = 4;
    tracker.params.initial_position_cov = 400.0;    // 与量测噪声量级接近
    tracker.params.initial_velocity_cov = 90000.0;  // 目标速度未知（300 m/s sigma）
    tracker.params.associator.gate_threshold = 25.0;

    // --- 交战：发射 + 制导 + 引信 ---
    engagement_controller engagement;
    engagement.lc.weapon.avg_thrusting_accel_mps2 = 200.0;
    engagement.lc.weapon.burnout_speed_mps        = 1000.0;
    engagement.lc.weapon.min_terminal_speed_mps   = 300.0;
    engagement.lc.limits.max_slant_range_m = 100000.0;
    engagement.lc.limits.max_time_of_flight_s = 180.0;
    engagement.fuze.pca.coarse_gate_m = 800.0;
    engagement.fuze.pca.fine_gate_m   = 80.0;
    engagement.fuze.fuze.trigger_radius_m = 80.0;
    engagement.fuze.fuze.arm_delay_s = 0.5;
    engagement.fuze.pk = pk_curve::blast_fragmentation(20.0, 80.0);

    // 弹体状态（一旦发射就用恒加速度近似推进）
    bool   weapon_launched = false;
    double weapon_launch_t = 0.0;
    vec3   weapon_pos{};
    vec3   weapon_vel{};
    int    engaged_track_id = -1;

    std::mt19937 rng(123);

    // --- 时间循环 ---
    double sim_time = 0.0;
    const double dt = 0.05;   // 20 Hz
    const double t_end = 120.0;

    printf(" t(s)  mode   R(km)  snr(dB)   Pd   trk  phase                \n");
    printf("-----  ----   -----  -------  ----  ---  ---------------------\n");

    int step = 0;
    double min_miss = 1e10;
    while (sim_time < t_end) {
        // 1) 推进目标真值
        target_pos = target_pos + target_vel * dt;

        // 2) 推进已发射的武器：沿速度方向推进，叠加 engagement 输出的侧向制导加速度。
        static vec3 last_guidance_accel{};
        if (weapon_launched) {
            double sp = weapon_vel.magnitude();
            vec3 fwd = (sp > 1e-3) ? weapon_vel / sp : (target_pos - weapon_pos).normalized();
            // 推力（沿速度方向，直到达到 burnout_speed）
            if (sp < engagement.lc.weapon.burnout_speed_mps) {
                weapon_vel = weapon_vel + fwd * (engagement.lc.weapon.avg_thrusting_accel_mps2 * dt);
            }
            // 侧向制导（由上一次 update 的 guidance_accel_cmd 给出）
            weapon_vel = weapon_vel + last_guidance_accel * dt;
            weapon_pos = weapon_pos + weapon_vel * dt;
        }

        // 3) 感知调度 + 探测
        auto cmd = scheduler.select(sim_time,
                                    weapon_launched ? sensor_mode::track : sensor_mode::search);
        detection_decision dec;
        std::vector<detection> detections;
        if (cmd.valid) {
            auto in = make_detection_inputs(target_pos);
            dec = sensor.evaluate(in);
            if (dec.detected) detections.push_back(make_detection(target_pos, rng));
        }

        // 4) 跟踪：仅在本 tick 传感器真正驻留时推进航迹（与 xsf-core 在 revisit 时触发 miss 的语义对齐）。
        track_manager_update_result track_res;
        if (cmd.valid) {
            track_res = tracker.update(detections, sim_time);
            for (int idx : track_res.unassociated_detection_indices) {
                tracker.start_tentative_track(detections[idx], sim_time);
            }
            for (int id : track_res.confirmed_track_ids) {
                scheduler.add_track_request(id, 0, sim_time);
                XSF_LOG_INFO("chain: track confirmed id={} at t={:.2f}s", id, sim_time);
            }
            for (int id : track_res.dropped_track_ids) {
                scheduler.drop_track_request(id);
                if (id == engaged_track_id) engaged_track_id = -1;
            }
        }

        // 5) 交战
        engagement_context ctx;
        ctx.sim_time_s = sim_time;
        ctx.weapon_launched = weapon_launched;
        // 从最近确认的航迹抓一条
        const track_record* rec = nullptr;
        for (const auto& kv : tracker.tracks) {
            if (kv.second.confirmed) { rec = &kv.second; break; }
        }
        engagement_command ecmd;
        if (rec) {
            if (!weapon_launched) {
                ctx.launch_cand.launcher_position_wcs = {0, 0, 0};
                ctx.launch_cand.target_position_wcs   = rec->kf.position();
                ctx.launch_cand.target_velocity_wcs   = rec->kf.velocity();
                ecmd = engagement.update(ctx);
                if (ecmd.issue_launch) {
                    weapon_launched = true;
                    weapon_launch_t = sim_time;
                    weapon_pos = {0, 0, 0};
                    weapon_vel = (rec->kf.position()).normalized() * 50.0;
                    engaged_track_id = rec->id;
                    printf("  %4.1f  LAUNCH track=%d intercept=[%.0f,%.0f,%.0f] TOF=%.1fs\n",
                           sim_time, rec->id,
                           ecmd.lc_result.intercept_point_wcs.x,
                           ecmd.lc_result.intercept_point_wcs.y,
                           ecmd.lc_result.intercept_point_wcs.z,
                           ecmd.lc_result.time_of_flight_s);
                }
            } else {
                ctx.geom.weapon_pos   = weapon_pos;
                ctx.geom.weapon_vel   = weapon_vel;
                ctx.geom.target_pos   = rec->kf.position();
                ctx.geom.target_vel   = rec->kf.velocity();
                ctx.fuze_in.flight_time_s = sim_time - weapon_launch_t;
                ctx.fuze_in.weapon_pos = weapon_pos;
                ctx.fuze_in.weapon_vel = weapon_vel;
                ctx.fuze_in.target_pos = target_pos;   // 引信看真值
                ctx.fuze_in.target_vel = target_vel;
                ctx.fuze_in.dt_s = dt;
                ecmd = engagement.update(ctx);
                last_guidance_accel = ecmd.guidance_accel_cmd;
            }
        }

        // 6) 打印：每秒最多一行；阶段切换时强制打印。
        static engagement_phase last_phase = engagement_phase::pre_engage;
        bool phase_changed = (ecmd.phase != last_phase);
        last_phase = ecmd.phase;
        if (cmd.valid && (step % 100 == 0 || phase_changed)) {
            const char* mode_str = (cmd.mode == sensor_mode::search) ? "SRCH" : "TRAK";
            const char* phase_str = "-";
            switch (ecmd.phase) {
                case engagement_phase::pre_engage:  phase_str = "pre_engage"; break;
                case engagement_phase::launch:      phase_str = "launch";     break;
                case engagement_phase::midcourse:   phase_str = "midcourse";  break;
                case engagement_phase::terminal:    phase_str = "terminal";   break;
                case engagement_phase::burst:       phase_str = "burst";      break;
                case engagement_phase::post_engage: phase_str = "post_engage";break;
            }
            printf("%5.2f  %s  %5.1f  %7.1f  %4.2f  %3zu  %s\n",
                   sim_time, mode_str, target_pos.magnitude() / 1000.0,
                   dec.snr_db, dec.pd, tracker.active_count(), phase_str);
        }

        if (weapon_launched) {
            double r = (target_pos - weapon_pos).magnitude();
            if (r < min_miss) min_miss = r;
        }

        // 7) 起爆或越过 CPA
        if (weapon_launched && ecmd.phase == engagement_phase::burst) {
            printf("\n  %4.1f  BURST  miss=%.2fm  eff_miss=%.2fm  Pk=%.3f\n",
                   sim_time,
                   ecmd.fuze_decision.miss_distance_m,
                   ecmd.fuze_decision.effective_miss_m,
                   ecmd.fuze_decision.estimated_pk);
            break;
        }
        if (weapon_launched && ecmd.phase == engagement_phase::post_engage) {
            printf("\n  %4.1f  BREAK-OFF (past CPA, no burst)\n", sim_time);
            break;
        }

        sim_time += dt;
        ++step;
    }

    printf("\nClosest approach distance: %.2f m\n", min_miss);
    printf("Active tracks at end: %zu\n", tracker.active_count());
    XSF_LOG_INFO("behavior chain example complete");
    return 0;
}
