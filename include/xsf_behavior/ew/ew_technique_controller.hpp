#pragma once

#include <xsf_common/log.hpp>
#include <xsf_math/ew/electronic_warfare.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace xsf_math {

// 电子战行为层：EW 技术库与选通控制器。
//
// 参考 xsf-core XsfEW_EA 的模式：
// - 技术库按 technique_id 注册；
// - 每条技术声明自身针对的系统功能（搜索 / 跟踪 / 引导 ...）与 mitigation class；
// - 控制器维护“当前已选通技术”集合；
// - 给定目标系统功能时，返回当前生效的降级乘子。
// 本模块不发射干扰，也不改变 EM 信道；它只聚合“这次应该用哪几种技术”。

enum class ew_system_function {
    unknown,
    search,
    track,
    guidance,
    communication
};

struct ew_technique {
    std::string         technique_id;
    std::string         mitigation_class_id;     // 对方 EP 通过该 ID 匹配反制
    ew_system_function  target_function = ew_system_function::unknown;
    ew_degradation      effect{};                 // 作用于基线 SNR 的乘子
};

struct ew_technique_controller {
    std::unordered_map<std::string, ew_technique> library;
    std::vector<std::string> selected_ids;

    bool add_technique(ew_technique t) {
        auto [it, inserted] = library.emplace(t.technique_id, std::move(t));
        if (!inserted) {
            XSF_LOG_WARN("ew controller: technique id already registered: {}", it->first);
        }
        return inserted;
    }

    bool select(const std::string& id, ew_system_function fn) {
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
        selected_ids.push_back(id);
        return true;
    }

    void deselect(const std::string& id) {
        for (auto it = selected_ids.begin(); it != selected_ids.end(); ++it) {
            if (*it == id) { selected_ids.erase(it); return; }
        }
    }

    // 合成当前生效的等效降级：乘子相乘、blanking_factor 取最小。
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

    std::size_t active_count() const { return selected_ids.size(); }
};

} // namespace xsf_math
