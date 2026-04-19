#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace xsf_math {

// CFAR 检测结果单元 (CFAR detection cell)
struct cfar_detection {
    std::size_t cell_index = 0; // 检测单元索引 (Cell index)
    double cell_power = 0.0;    // 单元功率 (Cell power)
    double threshold = 0.0;     // 检测门限 (Detection threshold)
};

// CFAR 处理结果 (CFAR processing result)
struct cfar_result {
    std::vector<double> thresholds;         // 各单元门限 (Per-cell thresholds)
    std::vector<cfar_detection> detections; // 过门限检测点 (Detections above threshold)
};

// 单元平均 CFAR 检测器（CA-CFAR）(Cell-averaging CFAR detector)
struct cell_averaging_cfar {
    int guard_cells = 2;       // 保护单元数 (Number of guard cells)
    int training_cells = 8;    // 参考单元数 (Number of training cells)
    double threshold_scale = 4.0; // 门限乘子 (Threshold scale factor)

    // 对功率单元序列执行 CFAR 检测 (Evaluate CFAR on power cell sequence)
    cfar_result evaluate(const std::vector<double>& power_cells) const {
        cfar_result out;
        out.thresholds.resize(power_cells.size(), 0.0);
        if (power_cells.empty()) return out;

        int window = guard_cells + training_cells;
        for (std::size_t i = 0; i < power_cells.size(); ++i) {
            if (static_cast<int>(i) < window ||
                static_cast<int>(i) + window >= static_cast<int>(power_cells.size())) {
                continue;
            }

            double sum = 0.0;
            int count = 0;
            for (int k = -window; k <= window; ++k) {
                if (std::abs(k) <= guard_cells) continue;
                sum += power_cells[static_cast<std::size_t>(static_cast<int>(i) + k)];
                ++count;
            }
            if (count <= 0) continue;

            double noise = sum / static_cast<double>(count);
            double threshold = noise * threshold_scale;
            out.thresholds[i] = threshold;
            if (power_cells[i] > threshold) {
                out.detections.push_back({i, power_cells[i], threshold});
            }
        }
        return out;
    }
};

// 多普勒模糊结果 (Doppler ambiguity result)
struct doppler_ambiguity_result {
    double unambiguous_velocity_mps = 0.0; // 最大不模糊速度（m/s）(Max unambiguous velocity)
    double blind_speed_mps = 0.0;          // 盲速（m/s）(Blind speed)
    double folded_velocity_mps = 0.0;      // 折叠后速度（m/s）(Folded/aliased velocity)
    double folded_frequency_hz = 0.0;      // 折叠后多普勒频率（Hz）(Folded Doppler frequency)
    int ambiguity_index = 0;               // 模糊数 (Ambiguity index)
};

// 评估多普勒模糊 (Evaluate Doppler ambiguity)
inline doppler_ambiguity_result evaluate_doppler_ambiguity(double radial_velocity_mps,
                                                           double carrier_frequency_hz,
                                                           double prf_hz) {
    doppler_ambiguity_result out;
    if (carrier_frequency_hz <= 0.0 || prf_hz <= 0.0) return out;

    double wavelength = constants::speed_of_light / carrier_frequency_hz;
    // 多普勒频率：f_d = 2 * v_r / lambda (Doppler frequency)
    double doppler_hz = 2.0 * radial_velocity_mps / wavelength;
    double folded = std::remainder(doppler_hz, prf_hz);

    out.folded_frequency_hz = folded;
    out.folded_velocity_mps = 0.5 * folded * wavelength;
    // 最大不模糊速度 = PRF * lambda / 4 (Unambiguous velocity)
    out.unambiguous_velocity_mps = 0.25 * prf_hz * wavelength;
    // 盲速 = PRF * lambda / 2 (Blind speed)
    out.blind_speed_mps = 0.5 * prf_hz * wavelength;
    out.ambiguity_index = static_cast<int>(std::llround((doppler_hz - folded) / prf_hz));
    return out;
}

}  // namespace xsf_math
