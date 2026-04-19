#pragma once

#include "../core/vec3.hpp"
#include "kalman_filter.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace xsf_math {

// IMM 模型配置（IMM model configuration）
struct imm_model_config {
    kalman_filter_6state filter{};
    double probability = 0.5;
};

// IMM 滤波器 6 状态（Interacting Multiple Model filter, 6-state）
struct imm_filter_6state {
    std::array<imm_model_config, 3> models{};
    std::size_t active_model_count = 2;
    double transition_probability[3][3] = {
        {0.90, 0.08, 0.02},
        {0.08, 0.84, 0.08},
        {0.02, 0.08, 0.90}
    };

    // 归一化模型概率（Normalize model probabilities）
    void normalize_probabilities() {
        double sum = 0.0;
        for (std::size_t i = 0; i < active_model_count; ++i) sum += models[i].probability;
        if (sum <= 1.0e-20) {
            double p = 1.0 / std::max<std::size_t>(active_model_count, 1);
            for (std::size_t i = 0; i < active_model_count; ++i) models[i].probability = p;
            return;
        }
        for (std::size_t i = 0; i < active_model_count; ++i) models[i].probability /= sum;
    }

    // 初始化所有模型（Initialize all models）
    void init(double time, const vec3& pos, const vec3& vel = {0, 0, 0}) {
        for (std::size_t i = 0; i < active_model_count; ++i) models[i].filter.init(time, pos, vel);
        normalize_probabilities();
    }

    // 设置指定模型的过程噪声（Set process noise profile for a model）
    void set_process_noise_profile(std::size_t idx, double qx, double qy, double qz) {
        if (idx >= active_model_count) return;
        models[idx].filter.process_noise[0] = qx;
        models[idx].filter.process_noise[1] = qy;
        models[idx].filter.process_noise[2] = qz;
    }

    // 预测步骤：先混合状态再各自预测（Prediction: mix states then predict each model）
    void predict(double time) {
        mix_states();
        for (std::size_t i = 0; i < active_model_count; ++i) models[i].filter.predict(time);
    }

    // 更新步骤：量测更新后重新估计概率（Update: measurement update then re-estimate probabilities）
    void update(double time, const vec3& meas_pos) {
        std::array<double, 3> likelihoods{};
        for (std::size_t i = 0; i < active_model_count; ++i) {
            auto& f = models[i].filter;
            f.update(time, meas_pos);
            vec3 residual = meas_pos - f.position();
            double sigma = std::sqrt(std::max(f.meas_noise[0], 1.0e-6));
            double dist2 = residual.magnitude_sq() / (sigma * sigma);
            likelihoods[i] = std::exp(-0.5 * dist2);
        }
        for (std::size_t i = 0; i < active_model_count; ++i) models[i].probability *= likelihoods[i];
        normalize_probabilities();
    }

    // 概率加权融合位置（Probability-weighted fused position）
    vec3 fused_position() const {
        vec3 out{};
        for (std::size_t i = 0; i < active_model_count; ++i) out += models[i].filter.position() * models[i].probability;
        return out;
    }

    // 概率加权融合速度（Probability-weighted fused velocity）
    vec3 fused_velocity() const {
        vec3 out{};
        for (std::size_t i = 0; i < active_model_count; ++i) out += models[i].filter.velocity() * models[i].probability;
        return out;
    }

    // 获取概率最大的模型索引（Get index of most likely model）
    std::size_t most_likely_model() const {
        std::size_t best = 0;
        for (std::size_t i = 1; i < active_model_count; ++i)
            if (models[i].probability > models[best].probability) best = i;
        return best;
    }

private:
    // 状态混合（交互）（Mix states across models）
    void mix_states() {
        std::array<double, 3> mixed_probs{};
        for (std::size_t j = 0; j < active_model_count; ++j)
            for (std::size_t i = 0; i < active_model_count; ++i)
                mixed_probs[j] += transition_probability[i][j] * models[i].probability;

        std::array<std::array<double, 6>, 3> mixed_states{};
        for (std::size_t j = 0; j < active_model_count; ++j) {
            if (mixed_probs[j] <= 1.0e-20) continue;
            for (std::size_t i = 0; i < active_model_count; ++i) {
                double mu = transition_probability[i][j] * models[i].probability / mixed_probs[j];
                for (int k = 0; k < 6; ++k) mixed_states[j][k] += mu * models[i].filter.state[k];
            }
        }

        for (std::size_t j = 0; j < active_model_count; ++j) {
            for (int k = 0; k < 6; ++k) models[j].filter.state[k] = mixed_states[j][k];
            models[j].probability = mixed_probs[j];
        }
        normalize_probabilities();
    }
};

}  // namespace xsf_math
