#pragma once

#include "../core/vec3.hpp"
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace xsf_math {

inline constexpr std::size_t invalid_target_index = std::numeric_limits<std::size_t>::max();

// 用于关联的航迹状态
struct track_state {
    int    id = -1;
    vec3   position;
    vec3   velocity;
    double position_covariance[3] = {100.0, 100.0, 100.0};  // 对角元素
};

// 检测量测
struct detection {
    vec3   position;
    double meas_noise[3] = {100.0, 100.0, 100.0};
    std::size_t target_index = invalid_target_index;  // 来自搜索/调度层的目标身份

    bool has_target_index() const { return target_index != invalid_target_index; }
};

// 关联结果
struct association_result {
    int track_id    = -1;   // -1 表示未关联
    int detection_idx = -1;
    double distance = 0.0;  // 马氏距离或欧氏距离
};

// 带门限的最近邻关联
struct nearest_neighbor_associator {

    double gate_threshold = 16.0;  // 卡方门限（3 自由度，16.0 约等于 99.7%）

    // 计算航迹预测位置与量测之间的马氏距离
    static double mahalanobis_distance(const track_state& trk,
                                       const detection& det) {
        double dx = det.position.x - trk.position.x;
        double dy = det.position.y - trk.position.y;
        double dz = det.position.z - trk.position.z;

        // 合成协方差 = 航迹协方差 + 量测噪声（对角）
        double sx = trk.position_covariance[0] + det.meas_noise[0];
        double sy = trk.position_covariance[1] + det.meas_noise[1];
        double sz = trk.position_covariance[2] + det.meas_noise[2];

        if (sx < 1e-20) sx = 1e-20;
        if (sy < 1e-20) sy = 1e-20;
        if (sz < 1e-20) sz = 1e-20;

        return (dx*dx/sx) + (dy*dy/sy) + (dz*dz/sz);
    }

    // 将量测与航迹关联（贪心最近邻）
    std::vector<association_result> associate(
            const std::vector<track_state>& tracks,
            const std::vector<detection>& detections) const {

        std::vector<association_result> results;
        std::vector<bool> det_used(detections.size(), false);

        for (const auto& trk : tracks) {
            double best_dist = gate_threshold;
            int best_det = -1;

            for (size_t d = 0; d < detections.size(); ++d) {
                if (det_used[d]) continue;
                double dist = mahalanobis_distance(trk, detections[d]);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_det = static_cast<int>(d);
                }
            }

            association_result r;
            r.track_id = trk.id;
            r.detection_idx = best_det;
            r.distance = best_dist;
            results.push_back(r);

            if (best_det >= 0) {
                det_used[static_cast<size_t>(best_det)] = true;
            }
        }

        // 输出未关联的量测
        for (size_t d = 0; d < detections.size(); ++d) {
            if (!det_used[d]) {
                association_result r;
                r.track_id = -1;
                r.detection_idx = static_cast<int>(d);
                r.distance = std::numeric_limits<double>::max();
                results.push_back(r);
            }
        }

        return results;
    }
};

// M-of-N 航迹确认逻辑
struct m_of_n_logic {
    int m = 3;  // 所需命中次数
    int n = 5;  // 窗口大小

    struct state {
        int hits = 0;
        int total = 0;
        bool confirmed = false;
    };

    void record_hit(state& s) const {
        s.hits++;
        s.total++;
        trim(s);
        s.confirmed = (s.hits >= m);
    }

    void record_miss(state& s) const {
        s.total++;
        trim(s);
        s.confirmed = (s.hits >= m);
    }

    bool is_confirmed(const state& s) const {
        return s.confirmed;
    }

private:
    void trim(state& s) const {
        // 简单滑动窗口近似
        while (s.total > n) {
            // 假设最旧条目按比例命中
            double hit_rate = (s.total > 0) ? static_cast<double>(s.hits) / s.total : 0.0;
            if (hit_rate > 0.5 && s.hits > 0) s.hits--;
            s.total--;
        }
    }
};

} // namespace xsf_math
