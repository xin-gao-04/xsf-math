#pragma once

#include <algorithm>
#include <limits>
#include <vector>

namespace xsf_math {

struct wta_target {
    int target_id = -1;
    double priority = 1.0;
};

struct wta_weapon {
    int weapon_id = -1;
    bool available = true;
};

struct wta_assignment {
    int weapon_id = -1;
    int target_id = -1;
    double score = 0.0;
};

struct weapon_assignment_result {
    std::vector<wta_assignment> assignments;
    double total_score = 0.0;
};

struct weapon_assignment_controller {
    // pk_matrix[w][t] 给出武器 w 对目标 t 的成功概率。
    weapon_assignment_result assign(const std::vector<wta_weapon>& weapons,
                                    const std::vector<wta_target>& targets,
                                    const std::vector<std::vector<double>>& pk_matrix) const {
        weapon_assignment_result out;
        std::vector<bool> target_taken(targets.size(), false);

        for (std::size_t w = 0; w < weapons.size(); ++w) {
            if (!weapons[w].available) continue;
            double best_score = -std::numeric_limits<double>::infinity();
            int best_target = -1;
            for (std::size_t t = 0; t < targets.size(); ++t) {
                if (target_taken[t]) continue;
                if (w >= pk_matrix.size() || t >= pk_matrix[w].size()) continue;
                double score = pk_matrix[w][t] * targets[t].priority;
                if (score > best_score) {
                    best_score = score;
                    best_target = static_cast<int>(t);
                }
            }
            if (best_target >= 0 && best_score > 0.0) {
                target_taken[static_cast<std::size_t>(best_target)] = true;
                out.assignments.push_back({weapons[w].weapon_id, targets[static_cast<std::size_t>(best_target)].target_id, best_score});
                out.total_score += best_score;
            }
        }
        return out;
    }
};

}  // namespace xsf_math
