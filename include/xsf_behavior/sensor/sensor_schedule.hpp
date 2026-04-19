#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

namespace xsf_math {

// 感知行为层：传感器调度器（Sensor behavior layer: sensor scheduler）。
//
// 参考 xsf-core XsfDefaultSensorScheduler 的语义：
// - 搜索模式下使用统计扫描，访问机会的时间由帧时间和机会间隔约束；
// - 跟踪模式下按请求列表逐条维护下一次访问时间；
// - 搜索与跟踪交替时，先服务到期的跟踪请求，再服务搜索。
// 本实现只输出“下一次应当照射哪个目标”的意图，不负责具体发射/接收链路。
// Reference xsf-core XsfDefaultSensorScheduler semantics:
// - Search mode uses statistical scan, visit opportunities constrained by frame time and interval;
// - Track mode maintains next visit time per request list entry;
// - When alternating search/track, serve due track requests first, then search.
// This implementation only outputs intent "which target to illuminate next", not actual TX/RX chain.

// 传感器模式（Sensor mode）
enum class sensor_mode {
    off,     // 关闭（Off）
    search,  // 搜索（Search）
    track    // 跟踪（Track）
};

// 传感器优先级策略（Sensor priority policy）
enum class sensor_priority_policy {
    track_first,     // 跟踪优先（Track first）
    threat_weighted  // 威胁加权（Threat weighted）
};

// 搜索槽：一个外部提示的目标索引，参与轮询扫描（Search slot: an externally hinted target index for polling scan）
struct sensor_search_slot {
    std::size_t target_index = 0;  // 外部目标列表的索引（Index into external target list）
    double priority_weight = 1.0;  // 优先级权重（Priority weight）
};

// 跟踪请求：一个已建立航迹的维持扫描任务（Track request: maintenance scan task for an established track）
struct sensor_track_request {
    int request_id = -1;              // 航迹 ID / 请求 ID（Track ID / request ID）
    std::size_t target_index = 0;     // 关联的目标索引（Associated target index）
    double next_visit_time_s = 0.0;   // 下一次应访问的仿真时间（Next scheduled visit simulation time）
    double last_visit_time_s = -1.0;  // 最近一次访问时间（-1 表示尚未访问）（Last visit time, -1 means not yet visited）
    double priority_weight = 2.0;  // 优先级权重（Priority weight）
};

// 调度器参数（Scheduler parameters）
struct sensor_schedule_params {
    double search_frame_time_s     = 2.0;    // 搜索一遍扫描列表所需时间（Time to scan entire search list）
    double track_revisit_time_s    = 0.5;    // 跟踪请求的默认再访问间隔（Default track revisit interval）
    double dwell_time_s            = 0.010;  // 单次驻留时间（Single dwell time）
    std::size_t max_track_requests = 16;     // 最大并发跟踪请求数（Maximum concurrent track requests）
    sensor_priority_policy priority_policy = sensor_priority_policy::track_first;  // 优先级策略（Priority policy）
    double track_priority_bias = 1.0;  // 跟踪优先级偏置（Track priority bias）
};

// 调度输出：告诉上层“下一次请点亮哪个目标、应如何配置扫描”（Schedule output: tells upper layer which target to illuminate next and scan configuration）
struct sensor_schedule_command {
    bool        valid               = false;           // 为假表示本周期无事可做（False means nothing to do this cycle）
    sensor_mode mode                = sensor_mode::off;  // 传感器模式（Sensor mode）
    std::size_t target_index        = 0;  // 目标索引（Target index）
    int         request_id          = -1;              // 仅跟踪模式有效（Valid only in track mode）
    double      scheduled_time_s    = 0.0;             // 预计访问时间（Scheduled visit time）
    double      dwell_time_s        = 0.0;             // 分配的驻留时间（Allocated dwell time）
};

// 调度器行为控制器（Scheduler behavior controller）。
// 维护搜索列表与跟踪请求列表，供上层按时间步驱动。
// Maintains search list and track request list, driven by upper layer time steps.
struct sensor_scheduler {
    sensor_schedule_params params{};

    // ---- 状态（State）----
    std::vector<sensor_search_slot>   search_list;  // 搜索列表（Search list）
    std::vector<sensor_track_request> track_list;  // 跟踪列表（Track list）
    std::size_t search_index = 0;  // 当前搜索索引（Current search index）
    double next_search_visit_s = 0.0;  // 下次搜索访问时间秒（Next search visit time in seconds）

    // 搜索机会间隔：帧时间 / 列表长度（参考 xsf-core UpdateSearchChanceInterval）（Search chance interval: frame time / list length）
    double search_chance_interval_s() const {
        std::size_t n = search_list.size();
        if (n == 0) return params.search_frame_time_s;
        return params.search_frame_time_s / static_cast<double>(n);
    }

    // 新增跟踪请求（M/N 确认后或外部提示建立新航迹时调用）（Add track request, called after M/N confirmation or external hint）
    bool add_track_request(int request_id, std::size_t target_index, double sim_time_s, double priority_weight = 2.0) {
        if (track_list.size() >= params.max_track_requests) {
            XSF_LOG_WARN("sensor scheduler: track list full, reject request_id={}", request_id);
            return false;
        }
        sensor_track_request r;
        r.request_id = request_id;
        r.target_index = target_index;
        r.next_visit_time_s = sim_time_s;
        r.priority_weight = priority_weight;
        track_list.push_back(r);
        return true;
    }

    // 丢弃跟踪请求（航迹丢失或超出能力）（Drop track request, track lost or beyond capacity）
    void drop_track_request(int request_id) {
        track_list.erase(std::remove_if(track_list.begin(), track_list.end(),
                                        [&](const sensor_track_request& r) { return r.request_id == request_id; }),
                         track_list.end());
    }

    // 选择当前时间片应当执行的下一次动作（Select next action for current time slice）。
    // 跟踪请求优先；若无到期跟踪，则按搜索列表轮询。
    // Track requests prioritized; if no due tracks, poll search list.
    sensor_schedule_command select(double sim_time_s, sensor_mode allowed_mode = sensor_mode::track) {
        sensor_schedule_command cmd{};
        if (allowed_mode == sensor_mode::off) return cmd;

        auto track_score = [&](const sensor_track_request& r) {
            double overdue = std::max(0.0, sim_time_s - r.next_visit_time_s);
            return params.track_priority_bias + r.priority_weight + overdue;
        };
        auto best_track = track_list.end();
        for (auto it = track_list.begin(); it != track_list.end(); ++it) {
            if (it->next_visit_time_s > sim_time_s) continue;
            if (best_track == track_list.end()) {
                best_track = it;
            } else if (params.priority_policy == sensor_priority_policy::threat_weighted) {
                if (track_score(*it) > track_score(*best_track)) best_track = it;
            } else if (it->next_visit_time_s < best_track->next_visit_time_s) {
                best_track = it;
            }
        }

        double search_score = -1.0;
        std::size_t search_pick = search_index;
        bool search_due = !search_list.empty() && sim_time_s >= next_search_visit_s;
        if (search_due && params.priority_policy == sensor_priority_policy::threat_weighted) {
            search_score = -1.0;
            for (std::size_t i = 0; i < search_list.size(); ++i) {
                if (search_list[i].priority_weight > search_score) {
                    search_score = search_list[i].priority_weight;
                    search_pick = i;
                }
            }
        }

        bool take_track = false;
        if (best_track != track_list.end()) {
            take_track = true;
            if (params.priority_policy == sensor_priority_policy::threat_weighted && search_due) {
                take_track = track_score(*best_track) >= search_score;
            }
        }

        if (take_track) {
            cmd.valid = true;
            cmd.mode  = sensor_mode::track;
            cmd.request_id = best_track->request_id;
            cmd.target_index = best_track->target_index;
            cmd.scheduled_time_s = sim_time_s;
            cmd.dwell_time_s = params.dwell_time_s;

            best_track->last_visit_time_s = sim_time_s;
            best_track->next_visit_time_s = sim_time_s + params.track_revisit_time_s;
            XSF_LOG_TRACE("sensor scheduler track: id={} t={:.3f}s revisit={:.3f}s",
                          cmd.request_id, cmd.scheduled_time_s, params.track_revisit_time_s);
            return cmd;
        }

        // 没有到期跟踪，按需服务搜索。
        if (allowed_mode == sensor_mode::search || allowed_mode == sensor_mode::track) {
            if (search_due) {
                if (search_index >= search_list.size()) search_index = 0;
                cmd.valid = true;
                cmd.mode  = sensor_mode::search;
                cmd.target_index = search_list[search_pick].target_index;
                cmd.scheduled_time_s = sim_time_s;
                cmd.dwell_time_s = params.dwell_time_s;

                search_index = (search_pick + 1) % search_list.size();
                next_search_visit_s = sim_time_s + search_chance_interval_s();
                XSF_LOG_TRACE("sensor scheduler search: target={} next_visit={:.3f}s",
                              cmd.target_index, next_search_visit_s);
            }
        }
        return cmd;
    }

    // 活跃跟踪请求数（Active track request count）
    std::size_t active_track_count() const { return track_list.size(); }
};

} // namespace xsf_math
