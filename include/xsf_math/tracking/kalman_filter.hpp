#pragma once

#include "../core/vec3.hpp"
#include <array>
#include <cmath>
#include <cstring>

namespace xsf_math {

// Lightweight 6-state Kalman filter for position/velocity tracking (WCS)
// State: [x, y, z, vx, vy, vz]
// Measurement: [x, y, z] (position only)
struct kalman_filter_6state {

    // State vector [x, y, z, vx, vy, vz]
    double state[6] = {};

    // State covariance P (6x6, symmetric, stored full)
    double P[6][6] = {};

    // Process noise spectral density (acceleration variance per axis)
    double process_noise[3] = {1.0, 1.0, 1.0};  // m^2/s^4

    // Measurement noise variance (position variance per axis)
    double meas_noise[3] = {100.0, 100.0, 100.0};  // m^2

    bool initialized = false;
    double last_time = 0.0;

    void reset() {
        std::memset(state, 0, sizeof(state));
        std::memset(P, 0, sizeof(P));
        initialized = false;
    }

    // Initialize with first measurement
    void init(double time, const vec3& pos, const vec3& vel = {0,0,0}) {
        state[0] = pos.x;  state[1] = pos.y;  state[2] = pos.z;
        state[3] = vel.x;  state[4] = vel.y;  state[5] = vel.z;

        // Initialize covariance
        std::memset(P, 0, sizeof(P));
        for (int i = 0; i < 3; ++i) {
            P[i][i]     = meas_noise[i];           // position uncertainty
            P[i+3][i+3] = meas_noise[i] * 4.0;     // velocity uncertainty (larger)
        }

        last_time = time;
        initialized = true;
    }

    // Predict step (constant velocity model)
    void predict(double time) {
        if (!initialized) return;

        double dt = time - last_time;
        if (dt <= 0.0) return;

        // State prediction: x = F * x (constant velocity)
        for (int i = 0; i < 3; ++i) {
            state[i] += state[i+3] * dt;
        }

        // Covariance prediction: P = F*P*F' + Q
        // F = [I dt*I; 0 I]
        // F*P*F' calculated explicitly
        double Pnew[6][6];
        std::memset(Pnew, 0, sizeof(Pnew));

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // Position-position block
                Pnew[i][j] = P[i][j] + dt * (P[i][j+3] + P[i+3][j]) + dt*dt * P[i+3][j+3];
                // Position-velocity block
                Pnew[i][j+3] = P[i][j+3] + dt * P[i+3][j+3];
                // Velocity-position block
                Pnew[i+3][j] = P[i+3][j] + dt * P[i+3][j+3];
                // Velocity-velocity block
                Pnew[i+3][j+3] = P[i+3][j+3];
            }
        }

        // Add process noise Q
        // Q derived from constant-acceleration model with spectral density q
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        for (int i = 0; i < 3; ++i) {
            Pnew[i][i]       += process_noise[i] * dt4 / 4.0;
            Pnew[i][i+3]     += process_noise[i] * dt3 / 2.0;
            Pnew[i+3][i]     += process_noise[i] * dt3 / 2.0;
            Pnew[i+3][i+3]   += process_noise[i] * dt2;
        }

        std::memcpy(P, Pnew, sizeof(P));
        last_time = time;
    }

    // Update step with position measurement
    void update(double time, const vec3& meas_pos) {
        if (!initialized) {
            init(time, meas_pos);
            return;
        }

        predict(time);

        // Innovation: y = z - H*x (H = [I 0])
        double y[3] = {
            meas_pos.x - state[0],
            meas_pos.y - state[1],
            meas_pos.z - state[2]
        };

        // Innovation covariance: S = H*P*H' + R (3x3)
        double S[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                S[i][j] = P[i][j];
            }
            S[i][i] += meas_noise[i];
        }

        // Invert S (3x3) using cofactor method
        double S_inv[3][3];
        invert_3x3(S, S_inv);

        // Kalman gain: K = P * H' * S^-1  (6x3)
        double K[6][3];
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 3; ++j) {
                K[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    K[i][j] += P[i][k] * S_inv[k][j];
                }
            }
        }

        // State update: x = x + K * y
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 3; ++j) {
                state[i] += K[i][j] * y[j];
            }
        }

        // Covariance update: P = (I - K*H) * P
        double Pnew[6][6];
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                Pnew[i][j] = P[i][j];
                for (int k = 0; k < 3; ++k) {
                    Pnew[i][j] -= K[i][k] * P[k][j];
                }
            }
        }
        std::memcpy(P, Pnew, sizeof(P));
    }

    // No-detect update (prediction only)
    void no_detect_update(double time) {
        predict(time);
    }

    // Get estimated position
    vec3 position() const { return {state[0], state[1], state[2]}; }

    // Get estimated velocity
    vec3 velocity() const { return {state[3], state[4], state[5]}; }

    // Chi-squared track score (normalized innovation squared)
    double track_score(const vec3& meas_pos) const {
        double y[3] = {
            meas_pos.x - state[0],
            meas_pos.y - state[1],
            meas_pos.z - state[2]
        };
        double S[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                S[i][j] = P[i][j];
            }
            S[i][i] += meas_noise[i];
        }
        double S_inv[3][3];
        invert_3x3(S, S_inv);

        double score = 0.0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                score += y[i] * S_inv[i][j] * y[j];
        return score;
    }

private:
    static void invert_3x3(const double m[3][3], double inv[3][3]) {
        double det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
                   - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
                   + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);

        if (std::abs(det) < 1e-30) {
            // Singular - return identity-like fallback
            std::memset(inv, 0, 9 * sizeof(double));
            for (int i = 0; i < 3; ++i) inv[i][i] = 1e10;
            return;
        }

        double inv_det = 1.0 / det;
        inv[0][0] =  (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * inv_det;
        inv[0][1] = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) * inv_det;
        inv[0][2] =  (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv_det;
        inv[1][0] = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]) * inv_det;
        inv[1][1] =  (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv_det;
        inv[1][2] = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) * inv_det;
        inv[2][0] =  (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * inv_det;
        inv[2][1] = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) * inv_det;
        inv[2][2] =  (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv_det;
    }
};

// Alpha-Beta filter (simpler, for smoothing noisy position data)
struct alpha_beta_filter {
    double alpha = 0.5;   // position smoothing (0-1)
    double beta  = 0.1;   // velocity smoothing (0-1)

    vec3 pos = {};
    vec3 vel = {};
    double last_time = 0.0;
    bool initialized = false;

    void reset() {
        pos = vel = {};
        initialized = false;
    }

    void update(double time, const vec3& meas_pos) {
        if (!initialized) {
            pos = meas_pos;
            vel = {};
            last_time = time;
            initialized = true;
            return;
        }

        double dt = time - last_time;
        if (dt <= 0.0) return;

        // Predict
        vec3 pos_pred = pos + vel * dt;

        // Residual
        vec3 residual = meas_pos - pos_pred;

        // Correct
        pos = pos_pred + residual * alpha;
        vel = vel + residual * (beta / dt);

        last_time = time;
    }

    vec3 predict_position(double time) const {
        double dt = time - last_time;
        return pos + vel * dt;
    }
};

} // namespace xsf_math
