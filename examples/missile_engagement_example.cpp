#include <xsf_math/xsf_math.hpp>
#include <xsf_common/log.hpp>
#include <xsf_common/validation_artifacts.hpp>
#include <cstdio>

using namespace xsf_math;

int main(int argc, char** argv) {
    const auto cli = xsf::validation::parse_cli(argc, argv);
    xsf::validation::case_artifacts artifacts("missile_engagement", cli);

    printf("=== Missile Engagement Simulation ===\n\n");

    // 目标：8km 高度的战斗机，向东飞行，马赫数 0.9
    double target_alt = 8000.0;
    double target_speed = atmosphere::speed_from_mach(target_alt, 0.9);
    vec3 target_pos = {0, 0, -target_alt};       // NED：z 轴向下
    vec3 target_vel = {0, target_speed, 0};       // 向东飞行

    // 导弹：从南侧发射，爬升中
    vec3 missile_pos = {-20000, 0, -5000};        // 南侧 20km，海拔 5km
    double missile_speed = atmosphere::speed_from_mach(5000.0, 2.5);
    vec3 missile_vel_dir = (target_pos - missile_pos).normalized();
    vec3 missile_vel = missile_vel_dir * missile_speed;

    // 制导
    augmented_proportional_nav apn;
    apn.nav_ratio = 4.0;

    // 气动
    aero_2d aero;
    aero.ref_area_m2 = 0.05;
    aero.cl_max = 20.0;
    aero.cd0 = 0.3;
    aero.aspect_ratio = 2.0;

    // 引信
    proximity_fuze fuze;
    fuze.trigger_radius_m = 15.0;
    fuze.arm_delay_s = 1.0;

    // 杀伤概率曲线
    auto pk = pk_curve::blast_fragmentation(10.0, 30.0);

    // ── 场景初始条件 ──────────────────────────────────────────────────────────
    {
        engagement_geometry g0;
        g0.weapon_pos = missile_pos;  g0.weapon_vel = missile_vel;
        g0.target_pos = target_pos;   g0.target_vel = target_vel;
        XSF_LOG_INFO("scenario start: missile Mach=2.5 alt={:.0f}m  "
                     "target Mach=0.9 alt={:.0f}m",
                     -missile_pos.z, target_alt);
        XSF_LOG_INFO("initial geometry: range={:.0f}m  Vc={:.1f}m/s  TTI={:.1f}s",
                     g0.slant_range(), g0.closing_velocity(), g0.time_to_intercept());
    }

    // 仿真
    double dt = 0.01;  // 10ms 时间步长
    double t = 0.0;
    double max_time = 60.0;
    bool terminal_logged = false;
    bool fuze_triggered = false;
    double terminal_range_m = -1.0;
    double trigger_time_s = -1.0;
    double miss_distance_m = -1.0;
    double pk_val = 0.0;
    double mc_kill_rate = 0.0;
    std::string final_outcome = "timeout";

    printf("Time(s)  Range(m)   Vc(m/s)  Accel(g)  Alt(m)\n");
    printf("-------  --------  --------  --------  ------\n");

    if (artifacts.enabled()) {
        artifacts.write_timeseries_header({
            "time_s", "range_m", "closing_velocity_mps", "accel_g", "altitude_m"
        });
    }

    while (t < max_time) {
        engagement_geometry geom;
        geom.weapon_pos = missile_pos;
        geom.weapon_vel = missile_vel;
        geom.target_pos = target_pos;
        geom.target_vel = target_vel;
        geom.target_accel = {0, 0, 0};

        double range = geom.slant_range();
        double vc = geom.closing_velocity();

        // 制导
        vec3 accel_cmd = apn.compute_accel(geom);

        // 限制加速度
        double q = atmosphere::dynamic_pressure(-missile_pos.z, missile_vel.magnitude());
        double max_g = accel_limiter::max_available_g(q, aero.ref_area_m2, aero.cl_max, 150.0);

        if (q < 500.0) {
            XSF_LOG_WARN("t={:.2f}s  dynamic pressure {:.0f} Pa is below 500 Pa — "
                         "guidance authority degraded", t, q);
        }

        accel_limiter limiter;
        limiter.max_g = max_g;
        vec3 accel = limiter.limit(accel_cmd);

        double accel_g = accel.magnitude() / constants::gravity_mps2;

        // 每 0.5s 打印一行并记录 DEBUG 日志
        if (std::fmod(t, 0.5) < dt) {
            printf("%5.1f    %8.0f  %8.1f  %8.1f  %6.0f\n",
                   t, range, vc, accel_g, -missile_pos.z);
            XSF_LOG_DEBUG("t={:.1f}s  range={:.0f}m  Vc={:.1f}m/s  "
                          "accel={:.1f}g  alt={:.0f}m",
                          t, range, vc, accel_g, -missile_pos.z);
            artifacts.append_timeseries_row({
                xsf::validation::make_field("", t, 3).value,
                xsf::validation::make_field("", range, 3).value,
                xsf::validation::make_field("", vc, 3).value,
                xsf::validation::make_field("", accel_g, 6).value,
                xsf::validation::make_field("", -missile_pos.z, 3).value
            });
        }

        // 末段提示（首次进入 3km）
        if (!terminal_logged && range < 3000.0) {
            XSF_LOG_INFO("terminal phase: range={:.0f}m  TTI={:.2f}s  "
                         "accel_cmd={:.1f}g",
                         range, geom.time_to_intercept(),
                         accel_cmd.magnitude() / constants::gravity_mps2);
            terminal_logged = true;
            terminal_range_m = range;
            artifacts.append_event(t,
                                   "terminal_phase",
                                   "Missile entered the terminal engagement phase.",
                                   {
                                       xsf::validation::make_field("range_m", range, 3),
                                       xsf::validation::make_field("tti_s", geom.time_to_intercept(), 3)
                                   });
        }

        // 引信检查
        auto fz = fuze.check(t, missile_pos, missile_vel, target_pos, target_vel, dt);
        if (fz == fuze_result::proximity_burst || fz == fuze_result::contact) {
            auto cpa = compute_cpa(missile_pos, missile_vel, target_pos, target_vel);
            double miss = cpa.miss_distance_m;
            pk_val = pk.evaluate(miss);
            miss_distance_m = miss;
            trigger_time_s = t;
            fuze_triggered = true;
            final_outcome = "fuze_triggered";

            XSF_LOG_INFO("fuze triggered at t={:.3f}s  miss={:.2f}m  Pk={:.3f}",
                         t, miss, pk_val);

            printf("\n--- FUZE TRIGGERED at t=%.3f s ---\n", t);
            printf("Miss distance: %.2f m\n", miss);
            printf("Pk: %.3f\n", pk_val);

            monte_carlo_kill mc(12345);
            int kills = 0, trials = 1000;
            for (int i = 0; i < trials; ++i) {
                monte_carlo_kill mc_trial(static_cast<unsigned>(i));
                if (mc_trial.evaluate(miss, pk)) kills++;
            }
            mc_kill_rate = static_cast<double>(kills) / static_cast<double>(trials);
            printf("Monte Carlo kill rate: %d/%d = %.1f%%\n", kills, trials,
                   100.0 * kills / trials);
            XSF_LOG_INFO("Monte Carlo: {}/{} = {:.1f}%", kills, trials,
                         100.0 * kills / trials);
            artifacts.append_event(t,
                                   "fuze_trigger",
                                   "Fuze triggered and evaluated kill probability.",
                                   {
                                       xsf::validation::make_field("miss_distance_m", miss, 3),
                                       xsf::validation::make_field("pk", pk_val, 6),
                                       xsf::validation::make_field("mc_kill_rate", mc_kill_rate, 6)
                                   });
            break;
        }

        if (fz == fuze_result::miss) {
            XSF_LOG_WARN("target missed at t={:.3f}s  final range={:.0f}m", t, range);
            printf("\n--- TARGET MISSED ---\n");
            final_outcome = "miss";
            artifacts.append_event(t,
                                   "miss",
                                   "Missile missed the target and ended the engagement.",
                                   {
                                       xsf::validation::make_field("range_m", range, 3)
                                   });
            break;
        }

        // 积分更新
        missile_vel += accel * dt;
        missile_pos += missile_vel * dt;

        // 目标做直线运动
        target_pos += target_vel * dt;

        t += dt;
    }

    const bool passed =
        fuze_triggered &&
        terminal_logged &&
        miss_distance_m >= 0.0 &&
        miss_distance_m <= fuze.trigger_radius_m &&
        pk_val >= 0.9;
    const std::string failure_reason = passed
        ? std::string()
        : "Missile engagement failed to reach terminal fuze success criteria";

    artifacts.write_metrics({
        xsf::validation::make_field("terminal_entered", terminal_logged),
        xsf::validation::make_field("terminal_range_m", terminal_range_m, 3),
        xsf::validation::make_field("fuze_triggered", fuze_triggered),
        xsf::validation::make_field("trigger_time_s", trigger_time_s, 3),
        xsf::validation::make_field("miss_distance_m", miss_distance_m, 3),
        xsf::validation::make_field("pk", pk_val, 6),
        xsf::validation::make_field("mc_kill_rate", mc_kill_rate, 6),
        xsf::validation::make_field("final_outcome", final_outcome)
    });
    artifacts.write_summary(
        passed,
        "Validate the end-to-end intercept chain from midcourse guidance into terminal fuze evaluation.",
        "Range should collapse smoothly, the scenario should enter terminal phase, and the fuze should trigger with high kill probability.",
        passed
            ? "The missile entered terminal guidance and triggered the fuze with near-zero miss distance and high kill probability."
            : "The intercept chain did not satisfy the expected terminal or fuze success criteria.",
        {
            xsf::validation::make_field("target_altitude_m", target_alt, 3),
            xsf::validation::make_field("target_speed_mps", target_speed, 3),
            xsf::validation::make_field("missile_speed_mps", missile_speed, 3),
            xsf::validation::make_field("fuze_radius_m", fuze.trigger_radius_m, 3)
        },
        {
            xsf::validation::make_field("terminal_entered", terminal_logged),
            xsf::validation::make_field("terminal_range_m", terminal_range_m, 3),
            xsf::validation::make_field("fuze_triggered", fuze_triggered),
            xsf::validation::make_field("trigger_time_s", trigger_time_s, 3),
            xsf::validation::make_field("miss_distance_m", miss_distance_m, 3),
            xsf::validation::make_field("pk", pk_val, 6),
            xsf::validation::make_field("mc_kill_rate", mc_kill_rate, 6),
            xsf::validation::make_field("final_outcome", final_outcome)
        },
        failure_reason);

    XSF_LOG_INFO("simulation complete");
    return (cli.strict && !passed) ? 1 : 0;
}
