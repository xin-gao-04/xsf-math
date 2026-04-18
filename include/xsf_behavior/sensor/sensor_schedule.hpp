#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/core/constants.hpp>
#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

namespace xsf_math {

// 感知行为层：传感器调度器。
//
// 参考 xsf-core XsfDefaultSensorScheduler 的语义：
// - 搜索模式下使用统计扫描，访问机会的时间由帧时间和机会间隔约束；
// - 跟踪模式下按请求列表逐条维护下一次访问时间；
// - 搜索与跟踪交替时，先服务到期的跟踪请求，再服务搜索。
// 本实现只输出“下一次应当照射哪个目标”的意图，不负责具体发射/接收链路。

enum class sensor_mode {
    off,
    search,
    track
};

// 搜索槽：一个外部提示的目标索引，参与轮询扫描。
struct sensor_search_slot {
    std::size_t target_index = 0;  // 外部目标列表的索引
};

// 跟踪请求：一个已建立航迹的维持扫描任务。
struct sensor_track_request {
    int request_id = -1;              // 航迹 ID / 请求 ID
    std::size_t target_index = 0;     // 关联的目标索引
    double next_visit_time_s = 0.0;   // 下一次应访问的仿真时间
    double last_visit_time_s = -1.0;  // 最近一次访问时间（-1 表示尚未访问）
};

// 调度器参数。
struct sensor_schedule_params {
    double search_frame_time_s     = 2.0;    // 搜索一遍扫描列表所需时间
    double track_revisit_time_s    = 0.5;    // 跟踪请求的默认再访问间隔
    double dwell_time_s            = 0.010;  // 单次驻留时间
    std::size_t max_track_requests = 16;     // 最大并发跟踪请求数
};

// 调度输出：告诉上层“下一次请点亮哪个目标、应如何配置扫描”。
struct sensor_schedule_command {
    bool        valid               = false;           // 为假表示本周期无事可做
    sensor_mode mode                = sensor_mode::off;
    std::size_t target_index        = 0;
    int         request_id          = -1;              // 仅跟踪模式有效
    double      scheduled_time_s    = 0.0;             // 预计访问时间
    double      dwell_time_s        = 0.0;             // 分配的驻留时间
};

// 调度器行为控制器。
// 维护搜索列表与跟踪请求列表，供上层按时间步驱动。
struct sensor_scheduler {
    sensor_schedule_params params{};

    // ---- 状态 ----
    std::vector<sensor_search_slot>   search_list;
    std::vector<sensor_track_request> track_list;
    std::size_t search_index = 0;
    double next_search_visit_s = 0.0;

    // 搜索机会间隔：帧时间 / 列表长度（参考 xsf-core UpdateSearchChanceInterval）。
    double search_chance_interval_s() const {
        std::size_t n = search_list.size();
        if (n == 0) return params.search_frame_time_s;
        return params.search_frame_time_s / static_cast<double>(n);
    }

    // 新增跟踪请求（M/N 确认后或外部提示建立新航迹时调用）。
    bool add_track_request(int request_id, std::size_t target_index, double sim_time_s) {
        if (track_list.size() >= params.max_track_requests) {
            XSF_LOG_WARN("sensor scheduler: track list full, reject request_id={}", request_id);
            return false;
        }
        sensor_track_request r;
        r.request_id = request_id;
        r.target_index = target_index;
        r.next_visit_time_s = sim_time_s;
        track_list.push_back(r);
        return true;
    }

    // 丢弃跟踪请求（航迹丢失或超出能力）。
    void drop_track_request(int request_id) {
        track_list.erase(std::remove_if(track_list.begin(), track_list.end(),
                                        [&](const sensor_track_request& r) { return r.request_id == request_id; }),
                         track_list.end());
    }

    // 选择当前时间片应当执行的下一次动作。
    // 跟踪请求优先；若无到期跟踪，则按搜索列表轮询。
    sensor_schedule_command select(double sim_time_s, sensor_mode allowed_mode = sensor_mode::track) {
        sensor_schedule_command cmd{};
        if (allowed_mode == sensor_mode::off) return cmd;

        // 优先处理到期的跟踪请求。
        auto it = std::min_element(track_list.begin(), track_list.end(),
                                   [](const sensor_track_request& a, const sensor_track_request& b) {
                                       return a.next_visit_time_s < b.next_visit_time_s;
                                   });
        if (it != track_list.end() && it->next_visit_time_s <= sim_time_s) {
            cmd.valid = true;
            cmd.mode  = sensor_mode::track;
            cmd.request_id = it->request_id;
            cmd.target_index = it->target_index;
            cmd.scheduled_time_s = sim_time_s;
            cmd.dwell_time_s = params.dwell_time_s;

            it->last_visit_time_s = sim_time_s;
            it->next_visit_time_s = sim_time_s + params.track_revisit_time_s;
            XSF_LOG_TRACE("sensor scheduler track: id={} t={:.3f}s revisit={:.3f}s",
                          cmd.request_id, cmd.scheduled_time_s, params.track_revisit_time_s);
            return cmd;
        }

        // 没有到期跟踪，按需服务搜索。
        if (allowed_mode == sensor_mode::search || allowed_mode == sensor_mode::track) {
            if (!search_list.empty() && sim_time_s >= next_search_visit_s) {
                if (search_index >= search_list.size()) search_index = 0;
                cmd.valid = true;
                cmd.mode  = sensor_mode::search;
                cmd.target_index = search_list[search_index].target_index;
                cmd.scheduled_time_s = sim_time_s;
                cmd.dwell_time_s = params.dwell_time_s;

                ++search_index;
                next_search_visit_s = sim_time_s + search_chance_interval_s();
                XSF_LOG_TRACE("sensor scheduler search: target={} next_visit={:.3f}s",
                              cmd.target_index, next_search_visit_s);
            }
        }
        return cmd;
    }

    std::size_t active_track_count() const { return track_list.size(); }
};

} // namespace xsf_math
