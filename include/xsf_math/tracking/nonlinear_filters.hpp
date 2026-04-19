#pragma once

#include "extended_kalman_filter.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <random>
#include <vector>

namespace xsf_math {

// 无迹卡尔曼滤波器 6 状态（Unscented Kalman filter, 6-state）
struct ukf_filter_6state {
    static constexpr int state_dim = 6;
    static constexpr int sigma_count = 2 * state_dim + 1;

    double state[state_dim] = {};
    double P[state_dim][state_dim] = {};
    double alpha = 0.3;  //  Sigma 点 spread 参数（Sigma point spread parameter）
    double beta = 2.0;  // 先验分布参数（Prior distribution parameter, optimal for Gaussian）
    double kappa = 0.0;  // 次级 scaling 参数（Secondary scaling parameter）
    double process_noise_accel = 2.0;  // 过程噪声加速度（Process noise acceleration）
    bool initialized = false;  // 是否已初始化（Whether initialized）
    double last_time_s = 0.0;  // 上次更新时间秒（Last update time in seconds）

    // 初始化（Initialize）
    void init(double time_s, const vec3& pos, const vec3& vel = {}) {
        state[0] = pos.x; state[1] = pos.y; state[2] = pos.z;
        state[3] = vel.x; state[4] = vel.y; state[5] = vel.z;
        std::memset(P, 0, sizeof(P));
        for (int i = 0; i < 3; ++i) {
            P[i][i] = 100.0;
            P[i + 3][i + 3] = 25.0;
        }
        initialized = true;
        last_time_s = time_s;
    }

    // 预测步骤（Prediction step）
    void predict(double time_s) {
        if (!initialized) return;
        double dt = time_s - last_time_s;
        if (dt <= 0.0) return;

        double sigma[sigma_count][state_dim];
        build_sigma_points(sigma);
        for (int k = 0; k < sigma_count; ++k) {
            sigma[k][0] += sigma[k][3] * dt;
            sigma[k][1] += sigma[k][4] * dt;
            sigma[k][2] += sigma[k][5] * dt;
        }
        recover_mean_and_covariance(sigma);

        double q_pos = process_noise_accel * dt * dt;
        double q_vel = process_noise_accel * dt;
        for (int i = 0; i < 3; ++i) {
            P[i][i] += q_pos;
            P[i + 3][i + 3] += q_vel;
        }
        last_time_s = time_s;
    }

    // 位置量测更新（Update with position measurement）
    void update_position(double time_s, const vec3& meas, double sigma_pos_m) {
        if (!initialized) {
            init(time_s, meas);
            return;
        }
        predict(time_s);

        double sigma[state_dim * 2 + 1][state_dim];
        build_sigma_points(sigma);
        double Z[sigma_count][3] = {};
        for (int k = 0; k < sigma_count; ++k) {
            Z[k][0] = sigma[k][0];
            Z[k][1] = sigma[k][1];
            Z[k][2] = sigma[k][2];
        }
        double z_mean[3] = {0.0, 0.0, 0.0};
        for (int k = 0; k < sigma_count; ++k) {
            double wm = mean_weight(k);
            z_mean[0] += wm * Z[k][0];
            z_mean[1] += wm * Z[k][1];
            z_mean[2] += wm * Z[k][2];
        }

        double S[3][3] = {};
        double Pxz[state_dim][3] = {};
        for (int k = 0; k < sigma_count; ++k) {
            double wc = covariance_weight(k);
            double dz[3] = {Z[k][0] - z_mean[0], Z[k][1] - z_mean[1], Z[k][2] - z_mean[2]};
            double dx[state_dim];
            for (int i = 0; i < state_dim; ++i) dx[i] = sigma[k][i] - state[i];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    S[i][j] += wc * dz[i] * dz[j];
            for (int i = 0; i < state_dim; ++i)
                for (int j = 0; j < 3; ++j)
                    Pxz[i][j] += wc * dx[i] * dz[j];
        }
        for (int i = 0; i < 3; ++i) S[i][i] += sigma_pos_m * sigma_pos_m;

        double S_inv[3][3];
        invert_3x3(S, S_inv);
        apply_update({meas.x - z_mean[0], meas.y - z_mean[1], meas.z - z_mean[2]}, Pxz, S, S_inv);
    }

    // 球坐标量测更新（Update with spherical measurement）
    void update_spherical(double time_s, const spherical_measurement& meas) {
        if (!initialized) {
            double ce = std::cos(meas.elevation_rad);
            vec3 pos = {
                meas.range_m * ce * std::cos(meas.azimuth_rad),
                meas.range_m * ce * std::sin(meas.azimuth_rad),
                -meas.range_m * std::sin(meas.elevation_rad)
            };
            init(time_s, pos);
            return;
        }
        predict(time_s);

        double sigma[sigma_count][state_dim];
        build_sigma_points(sigma);
        double Z[sigma_count][3] = {};
        double sin_az = 0.0, cos_az = 0.0, sin_el = 0.0, cos_el = 0.0;
        double z_mean[3] = {};
        for (int k = 0; k < sigma_count; ++k) {
            spherical_measurement z = measurement_from_state(sigma[k]);
            Z[k][0] = z.range_m;
            Z[k][1] = z.azimuth_rad;
            Z[k][2] = z.elevation_rad;
            double wm = mean_weight(k);
            z_mean[0] += wm * Z[k][0];
            sin_az += wm * std::sin(Z[k][1]);
            cos_az += wm * std::cos(Z[k][1]);
            sin_el += wm * std::sin(Z[k][2]);
            cos_el += wm * std::cos(Z[k][2]);
        }
        z_mean[1] = std::atan2(sin_az, cos_az);
        z_mean[2] = std::atan2(sin_el, cos_el);

        double S[3][3] = {};
        double Pxz[state_dim][3] = {};
        for (int k = 0; k < sigma_count; ++k) {
            double wc = covariance_weight(k);
            double dz[3] = {Z[k][0] - z_mean[0],
                            wrap_angle(Z[k][1] - z_mean[1]),
                            wrap_angle(Z[k][2] - z_mean[2])};
            double dx[state_dim];
            for (int i = 0; i < state_dim; ++i) dx[i] = sigma[k][i] - state[i];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    S[i][j] += wc * dz[i] * dz[j];
            for (int i = 0; i < state_dim; ++i)
                for (int j = 0; j < 3; ++j)
                    Pxz[i][j] += wc * dx[i] * dz[j];
        }
        S[0][0] += 100.0;
        S[1][1] += 1.0e-4;
        S[2][2] += 1.0e-4;

        double S_inv[3][3];
        invert_3x3(S, S_inv);
        apply_update({meas.range_m - z_mean[0],
                      wrap_angle(meas.azimuth_rad - z_mean[1]),
                      wrap_angle(meas.elevation_rad - z_mean[2])},
                     Pxz, S, S_inv);
    }

    // 获取估计位置（Get estimated position）
    vec3 position() const { return {state[0], state[1], state[2]}; }
    // 获取估计速度（Get estimated velocity）
    vec3 velocity() const { return {state[3], state[4], state[5]}; }

private:
    // 计算 lambda 参数（Compute lambda parameter）
    double lambda() const { return alpha * alpha * (state_dim + kappa) - state_dim; }
    // 计算 gamma 参数（Compute gamma parameter）
    double gamma() const { return std::sqrt(state_dim + lambda()); }
    // 计算均值权重（Compute mean weight）
    double mean_weight(int sigma_idx) const {
        return (sigma_idx == 0) ? lambda() / (state_dim + lambda())
                                : 1.0 / (2.0 * (state_dim + lambda()));
    }
    // 计算协方差权重（Compute covariance weight）
    double covariance_weight(int sigma_idx) const {
        return (sigma_idx == 0) ? mean_weight(0) + (1.0 - alpha * alpha + beta)
                                : mean_weight(sigma_idx);
    }

    // 构建 Sigma 点（Build sigma points）
    void build_sigma_points(double sigma[sigma_count][state_dim]) const {
        for (int k = 0; k < sigma_count; ++k)
            for (int i = 0; i < state_dim; ++i)
                sigma[k][i] = state[i];

        double g = gamma();
        for (int i = 0; i < state_dim; ++i) {
            double spread = g * std::sqrt(std::max(P[i][i], 1.0e-9));
            sigma[i + 1][i] += spread;
            sigma[i + 1 + state_dim][i] -= spread;
        }
    }

    // 从 Sigma 点恢复均值和协方差（Recover mean and covariance from sigma points）
    void recover_mean_and_covariance(const double sigma[sigma_count][state_dim]) {
        std::fill(std::begin(state), std::end(state), 0.0);
        for (int k = 0; k < sigma_count; ++k)
            for (int i = 0; i < state_dim; ++i)
                state[i] += mean_weight(k) * sigma[k][i];

        std::memset(P, 0, sizeof(P));
        for (int k = 0; k < sigma_count; ++k) {
            double wc = covariance_weight(k);
            for (int i = 0; i < state_dim; ++i) {
                double di = sigma[k][i] - state[i];
                for (int j = 0; j < state_dim; ++j) {
                    double dj = sigma[k][j] - state[j];
                    P[i][j] += wc * di * dj;
                }
            }
        }
    }

    // 从状态向量提取量测（Extract measurement from state）
    static spherical_measurement measurement_from_state(const double x[state_dim]) {
        return extended_kalman_filter_6state::state_to_measurement(x);
    }

    // 角度归一化到 [-pi, pi]（Wrap angle to [-pi, pi]）
    static double wrap_angle(double rad) {
        while (rad > constants::pi) rad -= constants::two_pi;
        while (rad < -constants::pi) rad += constants::two_pi;
        return rad;
    }

    // 应用卡尔曼更新（Apply Kalman update）
    void apply_update(const std::array<double, 3>& residual,
                      const double Pxz[state_dim][3],
                      const double S[3][3],
                      const double S_inv[3][3]) {
        double K[state_dim][3] = {};
        for (int i = 0; i < state_dim; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    K[i][j] += Pxz[i][k] * S_inv[k][j];

        for (int i = 0; i < state_dim; ++i)
            for (int j = 0; j < 3; ++j)
                state[i] += K[i][j] * residual[j];

        double KS[state_dim][3] = {};
        double KSKT[state_dim][state_dim] = {};
        for (int i = 0; i < state_dim; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    KS[i][j] += K[i][k] * S[k][j];
        for (int i = 0; i < state_dim; ++i)
            for (int j = 0; j < state_dim; ++j)
                for (int k = 0; k < 3; ++k)
                    KSKT[i][j] += KS[i][k] * K[j][k];

        for (int i = 0; i < state_dim; ++i)
            for (int j = 0; j < state_dim; ++j)
                P[i][j] -= KSKT[i][j];
    }

    // 3x3 矩阵求逆（Invert 3x3 matrix）
    static void invert_3x3(const double m[3][3], double inv[3][3]) {
        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if (std::abs(det) < 1.0e-20) {
            std::memset(inv, 0, 9 * sizeof(double));
            for (int i = 0; i < 3; ++i) inv[i][i] = 1.0e8;
            return;
        }
        double s = 1.0 / det;
        inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * s;
        inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * s;
        inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * s;
        inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * s;
        inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * s;
        inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * s;
        inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * s;
        inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * s;
        inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * s;
    }
};

// 粒子航迹状态（Particle track state）
struct particle_track_state {
    vec3 position{};
    vec3 velocity{};
    double weight = 0.0;
};

// 粒子滤波器 6 状态（Particle filter, 6-state）
struct particle_filter_6state {
    std::vector<particle_track_state> particles;
    double process_noise_position_m = 5.0;  // 位置过程噪声米（Position process noise in meters）
    double process_noise_velocity_mps = 1.0;  // 速度过程噪声 m/s（Velocity process noise in m/s）
    std::mt19937 rng{42u};  // 随机数生成器（Random number generator）

    // 初始化粒子群（Initialize particle set）
    void init(std::size_t count,
              const vec3& position,
              const vec3& velocity = {},
              double position_spread_m = 50.0,
              double velocity_spread_mps = 10.0) {
        particles.clear();
        particles.reserve(count);
        std::normal_distribution<double> pos_noise(0.0, position_spread_m);
        std::normal_distribution<double> vel_noise(0.0, velocity_spread_mps);
        double weight = (count == 0) ? 0.0 : 1.0 / static_cast<double>(count);
        for (std::size_t i = 0; i < count; ++i) {
            particles.push_back({
                {position.x + pos_noise(rng), position.y + pos_noise(rng), position.z + pos_noise(rng)},
                {velocity.x + vel_noise(rng), velocity.y + vel_noise(rng), velocity.z + vel_noise(rng)},
                weight
            });
        }
    }

    // 预测步骤（Prediction step）
    void predict(double dt_s) {
        if (dt_s <= 0.0) return;
        std::normal_distribution<double> pos_noise(0.0, process_noise_position_m);
        std::normal_distribution<double> vel_noise(0.0, process_noise_velocity_mps);
        for (auto& p : particles) {
            p.position += p.velocity * dt_s;
            p.position.x += pos_noise(rng);
            p.position.y += pos_noise(rng);
            p.position.z += pos_noise(rng);
            p.velocity.x += vel_noise(rng);
            p.velocity.y += vel_noise(rng);
            p.velocity.z += vel_noise(rng);
        }
    }

    // 位置量测更新（Update with position measurement）
    void update_position(const vec3& measurement, double sigma_pos_m) {
        if (particles.empty()) return;
        double sigma2 = std::max(sigma_pos_m * sigma_pos_m, 1.0e-6);
        double total = 0.0;
        for (auto& p : particles) {
            vec3 d = p.position - measurement;
            double likelihood = std::exp(-0.5 * d.magnitude_sq() / sigma2);
            p.weight *= likelihood;
            total += p.weight;
        }
        total = std::max(total, 1.0e-20);
        for (auto& p : particles) p.weight /= total;

        double ess_inv = 0.0;
        for (const auto& p : particles) ess_inv += p.weight * p.weight;
        double ess = 1.0 / std::max(ess_inv, 1.0e-20);
        if (ess < 0.5 * static_cast<double>(particles.size())) resample();
    }

    // 加权估计位置（Weighted estimated position）
    vec3 estimate_position() const {
        vec3 out;
        for (const auto& p : particles) out += p.position * p.weight;
        return out;
    }

    // 加权估计速度（Weighted estimated velocity）
    vec3 estimate_velocity() const {
        vec3 out;
        for (const auto& p : particles) out += p.velocity * p.weight;
        return out;
    }

private:
    // 重采样（低权重粒子替换）（Resample: replace low-weight particles）
    void resample() {
        if (particles.empty()) return;
        std::vector<particle_track_state> resampled;
        resampled.reserve(particles.size());

        std::vector<double> cdf(particles.size(), 0.0);
        cdf[0] = particles[0].weight;
        for (std::size_t i = 1; i < particles.size(); ++i) cdf[i] = cdf[i - 1] + particles[i].weight;

        std::uniform_real_distribution<double> dist(0.0, 1.0 / particles.size());
        double r = dist(rng);
        double step = 1.0 / particles.size();
        std::size_t idx = 0;
        for (std::size_t m = 0; m < particles.size(); ++m) {
            double u = r + m * step;
            while (idx + 1 < cdf.size() && u > cdf[idx]) ++idx;
            particle_track_state p = particles[idx];
            p.weight = step;
            resampled.push_back(p);
        }
        particles.swap(resampled);
    }
};

}  // namespace xsf_math
