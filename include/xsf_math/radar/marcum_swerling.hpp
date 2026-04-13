#pragma once

#include "../core/constants.hpp"
#include <cmath>
#include <algorithm>

namespace xsf_math {

// Marcum-Swerling 探测概率计算器
// 实现 Albersheim 近似，用于在不同检波律下计算 Swerling 0-4 的 Pd
struct marcum_swerling {

    enum class detector_law { linear, square, log };

    // 参数
    int          swerling_case            = 0;     // 0-4
    int          num_pulses_integrated    = 1;     // N >= 1
    double       prob_false_alarm         = 1.0e-6; // 虚警概率 Pfa
    detector_law law                      = detector_law::linear;

    // 根据线性 SNR 计算探测概率
    // detection_threshold 为线性值（不是 dB），默认约等于 3 dB
    double compute_pd(double snr_linear,
                      double detection_threshold = 1.9953) const {
        double base, exp;
        compute_constants(base, exp);

        if (snr_linear < 1.0e-50) return 0.0;
        if (snr_linear > 1.0e50)  return 1.0;

        double u = std::pow(base / snr_linear, exp);
        if (u <= 50.0)
            return std::pow(10.0, -u);
        return 0.0;
    }

    // 找到达到指定 Pd 所需的线性 SNR
    // 使用二分搜索
    double required_snr(double required_pd) const {
        double pd_clamped = std::clamp(required_pd, 0.002, 0.998);
        double lo = 0.0, hi = 1000.0;

        while (std::abs(hi - lo) > 0.001) {
            double mid = 0.5 * (lo + hi);
            double pd  = compute_pd(mid);
            if (std::abs(pd - pd_clamped) < 0.001) return mid;
            if (pd < pd_clamped) lo = mid;
            else                 hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    // 计算积分增益：单脉冲阈值与多脉冲阈值之比
    double integration_gain(double required_pd = 0.5) const {
        if (num_pulses_integrated <= 1) return 1.0;

        // 多脉冲阈值
        double multi_threshold = required_snr(required_pd);

        // 单脉冲阈值（同一检波器，N=1）
        marcum_swerling single = *this;
        single.num_pulses_integrated = 1;
        double single_threshold = single.required_snr(required_pd);

        if (multi_threshold < 1e-30) return 1.0;
        return single_threshold / multi_threshold;
    }

private:
    void compute_constants(double& out_base, double& out_exp) const {
        double pfa = prob_false_alarm;
        int    sc  = swerling_case;
        int    n   = num_pulses_integrated;

        // 单脉冲时将 2->1、4->3 进行映射
        if (n == 1) {
            if (sc == 2) sc = 1;
            else if (sc == 4) sc = 3;
        }

        double alp = 0.0, bet = 0.0, g1 = 0.0, g2 = 0.0, g3 = 0.0;

        switch (sc) {
        case 0:
            alp = 1.8; bet = 0.20; g1 = 1.2; g3 = 1.0;
            break;
        case 1:
            alp = (2.0/3.0) * (1.0 + (2.0/3.0) * std::exp(-n / 3.0));
            bet = 1.0; g1 = 1.0; g3 = 1.0;
            break;
        case 2:
            alp = 1.5 - std::log10(pfa) / 60.0;
            bet = (1.0/9.0) + std::exp(-n / 5.0);
            g1 = 0.5; g3 = 2.95;
            break;
        case 3:
            alp = (3.0/4.0) * (1.0 + (2.0/3.0) * std::exp(-n / 3.0));
            bet = 2.0/3.0; g1 = 0.93; g3 = 1.0;
            break;
        case 4:
            alp = 1.3 - std::log10(pfa) / 70.0;
            bet = (1.0/6.0) + (2.0/3.0) * std::exp(-n / 4.0);
            g1 = 0.83; g3 = 1.55;
            break;
        default:
            out_base = 1.0; out_exp = 1.0; return;
        }

        double ne = (n > 1) ? g1 * n : static_cast<double>(n);

        switch (law) {
        case detector_law::linear: g2 = 0.915; break;
        case detector_law::square: g2 = 1.0;   break;
        case detector_law::log:    g2 = 0.608; break;
        }

        double nfa   = std::log(0.5) / std::log(1.0 - pfa);
        double num   = alp * std::log10(nfa);
        double gama  = (2.0/3.0) * g2 * g1;
        double dem   = g3 * std::pow(ne, gama);

        out_base = num / dem;
        out_exp  = 1.0 / bet;
    }
};

// Albersheim 近似（另一种常用形式）
// 根据 Pd 和 Pfa 返回单脉冲所需的 SNR（dB）
inline double albersheim_snr_db(double pd, double pfa) {
    double A = std::log(0.62 / pfa);
    double B = std::log(pd / (1.0 - pd));
    return -5.0 * std::log10(6.2 + (4.54 / std::sqrt(1.0)) * std::sqrt(A + 0.12 * A * B + 1.7 * B));
}

// 带 N 脉冲积累的 Albersheim 近似
inline double albersheim_snr_db_n(double pd, double pfa, int n_pulses) {
    double snr1 = albersheim_snr_db(pd, pfa);
    // 积分增益近似
    double snr_n = snr1 - 10.0 * std::log10(static_cast<double>(n_pulses));
    return snr_n;
}

} // namespace xsf_math
