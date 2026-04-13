#include <xsf_math/xsf_math.hpp>
#include <cstdio>

using namespace xsf_math;

int main() {
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

    // 仿真
    double dt = 0.01;  // 10ms 时间步长
    double t = 0.0;
    double max_time = 60.0;

    printf("Time(s)  Range(m)   Vc(m/s)  Accel(g)  Alt(m)\n");
    printf("-------  --------  --------  --------  ------\n");

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
        accel_limiter limiter;
        limiter.max_g = max_g;
        vec3 accel = limiter.limit(accel_cmd);

        double accel_g = accel.magnitude() / constants::gravity_mps2;

        // 每 0.5s 输出一次
        if (std::fmod(t, 0.5) < dt) {
            printf("%5.1f    %8.0f  %8.1f  %8.1f  %6.0f\n",
                   t, range, vc, accel_g, -missile_pos.z);
        }

        // 引信检查
        auto fz = fuze.check(t, missile_pos, missile_vel, target_pos, target_vel, dt);
        if (fz == fuze_result::proximity_burst || fz == fuze_result::contact) {
            auto cpa = compute_cpa(missile_pos, missile_vel, target_pos, target_vel);
            double miss = cpa.miss_distance_m;
            double pk_val = pk.evaluate(miss);
            printf("\n--- FUZE TRIGGERED at t=%.3f s ---\n", t);
            printf("Miss distance: %.2f m\n", miss);
            printf("Pk: %.3f\n", pk_val);

            monte_carlo_kill mc(12345);
            int kills = 0, trials = 1000;
            for (int i = 0; i < trials; ++i) {
                monte_carlo_kill mc_trial(static_cast<unsigned>(i));
                if (mc_trial.evaluate(miss, pk)) kills++;
            }
            printf("Monte Carlo kill rate: %d/%d = %.1f%%\n", kills, trials,
                   100.0 * kills / trials);
            break;
        }

        if (fz == fuze_result::miss) {
            printf("\n--- TARGET MISSED ---\n");
            break;
        }

        // 积分更新
        missile_vel += accel * dt;
        missile_pos += missile_vel * dt;

        // 目标做直线运动
        target_pos += target_vel * dt;

        t += dt;
    }

    return 0;
}
