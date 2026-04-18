#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/tracking/kalman_filter.hpp>
#include <xsf_math/tracking/track_association.hpp>
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace xsf_math {

// 跟踪行为层：航迹管理控制器。
//
// 参考 xsf-core XsfDefaultSensorTracker::State 的组织：
// - 每条活跃航迹维护一个独立滤波器 + 检测历史位流 + 失联计数；
// - 确认阶段走 M/N 逻辑；
// - 失联达到阈值后丢弃。
// 本模块不负责调度，只负责“量测来了以后如何把它落到航迹上”。

struct track_record {
    int                   id = -1;
    kalman_filter_6state  kf{};
    m_of_n_logic::state   mofn{};
    int                   failures_until_drop = -1;  // -1 表示未初始化
    unsigned int          detection_history_bits = 0;
    double                last_update_time_s = 0.0;
    bool                  confirmed = false;
    std::size_t           target_index = invalid_target_index;

    bool has_target_index() const { return target_index != invalid_target_index; }
};

struct track_manager_params {
    m_of_n_logic             mofn{};                   // 默认 m=3, n=5
    nearest_neighbor_associator associator{};           // 含 gate_threshold
    int                       drop_after_misses = 5;   // 连续未命中丢航迹
    double                    initial_position_cov = 100.0;
    double                    initial_velocity_cov = 25.0;
};

struct track_manager_update_result {
    struct confirmed_track_binding {
        int         track_id = -1;
        std::size_t target_index = invalid_target_index;

        bool has_target_index() const { return target_index != invalid_target_index; }
    };

    std::vector<int>                unassociated_detection_indices;
    std::vector<int>                dropped_track_ids;
    std::vector<int>                confirmed_track_ids;   // 本次更新后新确认的航迹
    std::vector<confirmed_track_binding> confirmed_tracks; // 兼容 scheduler 所需的身份回传
};

struct track_manager {
    track_manager_params params{};

    // ---- 状态 ----
    std::unordered_map<int, track_record> tracks;
    int next_track_id = 1;

    // 从一个新量测起始一条试探航迹。
    int start_tentative_track(const detection& det, double sim_time_s, const vec3& initial_velocity = {}) {
        track_record rec;
        rec.id = next_track_id++;
        rec.target_index = det.target_index;
        for (int i = 0; i < 3; ++i) {
            rec.kf.meas_noise[i] = params.initial_position_cov;
        }
        rec.kf.init(sim_time_s, det.position, initial_velocity);
        // 速度协方差由 init 默认填 4 × meas_noise，若需要外部可覆盖。
        for (int i = 0; i < 3; ++i) {
            rec.kf.P[i+3][i+3] = params.initial_velocity_cov;
        }
        rec.last_update_time_s = sim_time_s;
        rec.failures_until_drop = params.drop_after_misses;
        params.mofn.record_hit(rec.mofn);
        tracks.emplace(rec.id, std::move(rec));
        XSF_LOG_DEBUG("track manager: start tentative id={} at t={:.3f}s", next_track_id - 1, sim_time_s);
        return next_track_id - 1;
    }

    // 按新一组量测推进所有航迹。
    // 流程：(1) 对每条航迹预测；(2) 组装 track_state 与 detection 交给 associator；
    // (3) 对每条航迹喂命中或失配；(4) 淘汰超时未命中的航迹。
    track_manager_update_result update(const std::vector<detection>& detections,
                                        double sim_time_s) {
        track_manager_update_result result;

        // 1) 预测步，并构造 track_state 列表
        std::vector<track_state> track_states;
        std::vector<int>         track_ids_ordered;
        track_states.reserve(tracks.size());
        track_ids_ordered.reserve(tracks.size());
        for (auto& kv : tracks) {
            auto& rec = kv.second;
            if (sim_time_s > rec.kf.last_time) rec.kf.predict(sim_time_s);
            track_state ts;
            ts.id = rec.id;
            ts.position = rec.kf.position();
            ts.velocity = rec.kf.velocity();
            ts.position_covariance[0] = rec.kf.P[0][0];
            ts.position_covariance[1] = rec.kf.P[1][1];
            ts.position_covariance[2] = rec.kf.P[2][2];
            track_states.push_back(ts);
            track_ids_ordered.push_back(rec.id);
        }

        // 2) 关联
        auto assoc = params.associator.associate(track_states, detections);

        // 3) 按关联结果更新滤波器 + M/N
        std::vector<bool> detection_used(detections.size(), false);
        for (const auto& a : assoc) {
            if (a.track_id < 0) continue;  // 未关联量测留到后面
            auto it = tracks.find(a.track_id);
            if (it == tracks.end()) continue;
            auto& rec = it->second;
            if (a.detection_idx >= 0) {
                const auto& det = detections[static_cast<std::size_t>(a.detection_idx)];
                rec.kf.update(sim_time_s, det.position);
                if (det.has_target_index()) rec.target_index = det.target_index;
                params.mofn.record_hit(rec.mofn);
                rec.failures_until_drop = params.drop_after_misses;
                rec.detection_history_bits = (rec.detection_history_bits << 1) | 0x1u;
                detection_used[static_cast<std::size_t>(a.detection_idx)] = true;
                bool was_confirmed = rec.confirmed;
                rec.confirmed = params.mofn.is_confirmed(rec.mofn);
                if (!was_confirmed && rec.confirmed) {
                    result.confirmed_track_ids.push_back(rec.id);
                    result.confirmed_tracks.push_back({rec.id, rec.target_index});
                }
            } else {
                params.mofn.record_miss(rec.mofn);
                if (rec.failures_until_drop > 0) --rec.failures_until_drop;
                rec.detection_history_bits <<= 1;
            }
            rec.last_update_time_s = sim_time_s;
        }

        // 4) 汇报未关联量测 + 丢弃超时航迹
        for (std::size_t d = 0; d < detections.size(); ++d) {
            if (!detection_used[d]) result.unassociated_detection_indices.push_back(static_cast<int>(d));
        }
        for (auto it = tracks.begin(); it != tracks.end();) {
            if (it->second.failures_until_drop == 0) {
                result.dropped_track_ids.push_back(it->first);
                XSF_LOG_INFO("track manager: drop id={} t={:.3f}s", it->first, sim_time_s);
                it = tracks.erase(it);
            } else {
                ++it;
            }
        }
        return result;
    }

    // 查询接口。
    const track_record* find(int track_id) const {
        auto it = tracks.find(track_id);
        return (it == tracks.end()) ? nullptr : &it->second;
    }
    std::size_t active_count() const { return tracks.size(); }
};

} // namespace xsf_math
