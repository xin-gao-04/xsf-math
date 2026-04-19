#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include "kalman_filter.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace xsf_math {

// 球坐标量测（Spherical measurement）
struct spherical_measurement {
    double range_m = 0.0;
    double azimuth_rad = 0.0;
    double elevation_rad = 0.0;
};

// 扩展卡尔曼滤波器 6 状态（Extended Kalman filter, 6-state）
struct extended_kalman_filter_6state {
    double state[6] = {};
    double P[6][6] = {};
    double process_noise[3] = {1.0, 1.0, 1.0};  // 过程噪声（Process noise）
    double meas_noise[3] = {100.0, 1.0e-4, 1.0e-4};  // range / az / el 量测噪声（Measurement noise: range / azimuth / elevation）
    bool initialized = false;  // 是否已初始化（Whether initialized）
    double last_time = 0.0;  // 上次更新时间（Last update time）

    // 初始化（Initialize）
    void init(double time, const vec3& pos, const vec3& vel = {0, 0, 0}) {
        state[0] = pos.x; state[1] = pos.y; state[2] = pos.z;
        state[3] = vel.x; state[4] = vel.y; state[5] = vel.z;
        std::memset(P, 0, sizeof(P));
        for (int i = 0; i < 3; ++i) {
            P[i][i] = meas_noise[0];
            P[i + 3][i + 3] = 4.0 * meas_noise[0];
        }
        initialized = true;
        last_time = time;
    }

    // 预测步骤（Prediction step）
    void predict(double time) {
        if (!initialized) return;
        double dt = time - last_time;
        if (dt <= 0.0) return;
        for (int i = 0; i < 3; ++i) state[i] += state[i + 3] * dt;

        double Pnew[6][6];
        std::memset(Pnew, 0, sizeof(Pnew));
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Pnew[i][j] = P[i][j] + dt * (P[i][j + 3] + P[i + 3][j]) + dt * dt * P[i + 3][j + 3];
                Pnew[i][j + 3] = P[i][j + 3] + dt * P[i + 3][j + 3];
                Pnew[i + 3][j] = P[i + 3][j] + dt * P[i + 3][j + 3];
                Pnew[i + 3][j + 3] = P[i + 3][j + 3];
            }
        }

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        for (int i = 0; i < 3; ++i) {
            Pnew[i][i] += process_noise[i] * dt4 / 4.0;
            Pnew[i][i + 3] += process_noise[i] * dt3 / 2.0;
            Pnew[i + 3][i] += process_noise[i] * dt3 / 2.0;
            Pnew[i + 3][i + 3] += process_noise[i] * dt2;
        }
        std::memcpy(P, Pnew, sizeof(P));
        last_time = time;
    }

    // 状态向量转球坐标量测（Convert state to spherical measurement）
    static spherical_measurement state_to_measurement(const double x[6]) {
        spherical_measurement z;
        double px = x[0], py = x[1], pz = x[2];
        double ground = std::sqrt(px * px + py * py);
        z.range_m = std::sqrt(px * px + py * py + pz * pz);
        z.azimuth_rad = std::atan2(py, px);
        z.elevation_rad = std::atan2(-pz, std::max(ground, 1.0e-12));
        return z;
    }

    // 计算量测雅可比矩阵（Compute measurement Jacobian）
    static void measurement_jacobian(const double x[6], double H[3][6]) {
        std::memset(H, 0, 18 * sizeof(double));
        double px = x[0], py = x[1], pz = x[2];
        double r2 = px * px + py * py + pz * pz;
        double rho2 = px * px + py * py;
        double r = std::sqrt(std::max(r2, 1.0e-12));
        double rho = std::sqrt(std::max(rho2, 1.0e-12));

        H[0][0] = px / r; H[0][1] = py / r; H[0][2] = pz / r;
        H[1][0] = -py / std::max(rho2, 1.0e-12);
        H[1][1] =  px / std::max(rho2, 1.0e-12);
        H[2][0] =  px * pz / (std::max(r2, 1.0e-12) * rho);
        H[2][1] =  py * pz / (std::max(r2, 1.0e-12) * rho);
        H[2][2] = -rho / std::max(r2, 1.0e-12);
    }

    // 球坐标量测更新（Update with spherical measurement）
    void update_spherical(double time, const spherical_measurement& meas) {
        if (!initialized) {
            double ce = std::cos(meas.elevation_rad);
            vec3 pos = {
                meas.range_m * ce * std::cos(meas.azimuth_rad),
                meas.range_m * ce * std::sin(meas.azimuth_rad),
                -meas.range_m * std::sin(meas.elevation_rad)
            };
            init(time, pos);
            return;
        }

        predict(time);

        spherical_measurement hx = state_to_measurement(state);
        double y[3] = {
            meas.range_m - hx.range_m,
            meas.azimuth_rad - hx.azimuth_rad,
            meas.elevation_rad - hx.elevation_rad
        };
        while (y[1] > constants::pi) y[1] -= constants::two_pi;
        while (y[1] < -constants::pi) y[1] += constants::two_pi;

        double H[3][6];
        measurement_jacobian(state, H);

        double HP[3][6] = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    HP[i][j] += H[i][k] * P[k][j];

        double S[3][3] = {};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 6; ++k)
                    S[i][j] += HP[i][k] * H[j][k];
            S[i][i] += meas_noise[i];
        }

        double S_inv[3][3];
        invert_3x3(S, S_inv);

        double K[6][3] = {};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    K[i][j] += P[i][k] * S_inv[k][j];

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                state[i] += K[i][j] * y[j];

        double KH[6][6] = {};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 3; ++k)
                    KH[i][j] += K[i][k] * H[k][j];

        double Pnew[6][6] = {};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                double I_minus_KH = (i == j ? 1.0 : 0.0) - KH[i][j];
                for (int k = 0; k < 6; ++k) Pnew[i][j] += I_minus_KH * P[k][j] * (k == i ? 1.0 : 0.0);
            }
        }
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                Pnew[i][j] = P[i][j];
                for (int k = 0; k < 3; ++k) Pnew[i][j] -= K[i][k] * HP[k][j];
            }
        std::memcpy(P, Pnew, sizeof(P));
    }

    // 获取估计位置（Get estimated position）
    vec3 position() const { return {state[0], state[1], state[2]}; }
    // 获取估计速度（Get estimated velocity）
    vec3 velocity() const { return {state[3], state[4], state[5]}; }

private:
    // 3x3 矩阵求逆（Invert 3x3 matrix）
    static void invert_3x3(const double m[3][3], double inv[3][3]) {
        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if (std::abs(det) < 1e-20) {
            std::memset(inv, 0, 9 * sizeof(double));
            for (int i = 0; i < 3; ++i) inv[i][i] = 1e8;
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

}  // namespace xsf_math
