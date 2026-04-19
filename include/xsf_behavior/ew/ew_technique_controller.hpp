#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/ew/electronic_warfare.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace xsf_math {

// 电子战行为层：EW 技术库与选通控制器（Electronic warfare behavior layer: EW technique library and gating controller）。
//
// 参考 xsf-core XsfEW_EA 的模式：
// - 技术库按 technique_id 注册；
// - 每条技术声明自身针对的系统功能（搜索 / 跟踪 / 引导 ...）与 mitigation class；
// - 控制器维护“当前已选通技术”集合；
// - 给定目标系统功能时，返回当前生效的降级乘子。
// 本模块不发射干扰，也不改变 EM 信道；它只聚合“这次应该用哪几种技术”。
// Reference xsf-core XsfEW_EA pattern:
// - Technique library registered by technique_id;
// - Each technique declares target system function and mitigation class;
// - Controller maintains currently selected technique set;
// - Returns effective degradation multiplier for given target system function.
// This module does not transmit jamming or change EM channels; it only aggregates which techniques to use.

// EW 目标系统功能（EW target system function）
enum class ew_system_function {
    unknown,     // 未知（Unknown）
    search,      // 搜索（Search）
    track,       // 跟踪（Track）
    guidance,    // 引导（Guidance）
    communication // 通信（Communication）
};

// EW 技术条目（EW technique entry）
struct ew_technique {
    std::string         technique_id;  // 技术标识（Technique ID）
    std::string         mitigation_class_id;     // 对方 EP 通过该 ID 匹配反制（Opponent EP matches countermeasure by this ID）
    std::string         mutex_group_id;          // 同组技术互斥（Mutex group ID, techniques in same group are mutually exclusive）
    ew_system_function  target_function = ew_system_function::unknown;  // 目标功能（Target function）
    ew_degradation      effect{};                 // 作用于基线 SNR 的乘子（Degradation effect multiplier on baseline SNR）
    double              resource_cost = 1.0;  // 资源消耗（Resource cost）
    double              min_hold_time_s = 0.0;  // 最小保持时间秒（Minimum hold time in seconds）
};

// EW 技术控制器（EW technique controller）
struct ew_technique_controller {
    std::unordered_map<std::string, ew_technique> library;  // 技术库（Technique library）
    std::vector<std::string> selected_ids;  // 已选技术 ID 列表（Selected technique IDs）
    double max_resource_budget = 4.0;  // 最大资源预算（Maximum resource budget）
    std::unordered_map<std::string, double> selection_time_s;  // 各技术的选中时间秒（Selection time per technique in seconds）

    // 添加技术到库（Add technique to library）
    bool add_technique(ew_technique t) {
        auto [it, inserted] = library.emplace(t.technique_id, std::move(t));
        if (!inserted) {
            XSF_LOG_WARN("ew controller: technique id already registered: {}", it->first);
        }
        return inserted;
    }

    // 当前资源消耗（Current resource cost）
    double current_resource_cost() const {
        double total = 0.0;
        for (const auto& id : selected_ids) {
            auto it = library.find(id);
            if (it != library.end()) total += it->second.resource_cost;
        }
        return total;
    }

    // 选通技术（Select technique）
    bool select(const std::string& id, ew_system_function fn, double sim_time_s = 0.0) {
        auto it = library.find(id);
        if (it == library.end()) {
            XSF_LOG_WARN("ew controller: select unknown technique {}", id);
            return false;
        }
        if (it->second.target_function != ew_system_function::unknown &&
            it->second.target_function != fn) {
            XSF_LOG_DEBUG("ew controller: technique {} not applicable to function {}",
                          id, static_cast<int>(fn));
            return false;
        }
        for (const auto& s : selected_ids) if (s == id) return true;
        if (!it->second.mutex_group_id.empty()) {
            for (const auto& s : selected_ids) {
                auto active = library.find(s);
                if (active != library.end() &&
                    active->second.mutex_group_id == it->second.mutex_group_id) {
                    return false;
                }
            }
        }
        if (current_resource_cost() + it->second.resource_cost > max_resource_budget) return false;
        selected_ids.push_back(id);
        selection_time_s[id] = sim_time_s;
        return true;
    }

    // 取消选通技术（Deselect technique）
    void deselect(const std::string& id, double sim_time_s = 0.0) {
        for (auto it = selected_ids.begin(); it != selected_ids.end(); ++it) {
            if (*it != id) continue;
            auto lib_it = library.find(id);
            auto ts_it = selection_time_s.find(id);
            if (lib_it != library.end() && ts_it != selection_time_s.end()) {
                if (sim_time_s - ts_it->second < lib_it->second.min_hold_time_s) return;
            }
            selected_ids.erase(it);
            selection_time_s.erase(id);
            return;
        }
    }

    // 合成当前生效的等效降级：乘子相乘、blanking_factor 取最小。
    // 合成当前生效的等效降级（Compute combined effective degradation: multiply gains, take minimum blanking factor）
    ew_degradation combined_effect_for(ew_system_function fn) const {
        ew_degradation out{};
        for (const auto& id : selected_ids) {
            auto it = library.find(id);
            if (it == library.end()) continue;
            if (it->second.target_function != ew_system_function::unknown &&
                it->second.target_function != fn) continue;
            const auto& e = it->second.effect;
            out.jamming_power_gain *= e.jamming_power_gain;
            out.noise_multiplier   *= e.noise_multiplier;
            if (e.blanking_factor < out.blanking_factor) out.blanking_factor = e.blanking_factor;
        }
        return out;
    }

    // 活跃技术数量（Active technique count）
    std::size_t active_count() const { return selected_ids.size(); }
};

} // namespace xsf_math
