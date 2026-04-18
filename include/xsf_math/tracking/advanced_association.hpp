#pragma once

#include "track_association.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace xsf_math {

struct association_hypothesis {
    std::vector<association_result> assignments;
    double total_cost = 0.0;
};

struct jpda_pair_probability {
    int track_id = -1;
    int detection_idx = -1;
    double probability = 0.0;
};

struct jpda_result {
    std::vector<jpda_pair_probability> marginals;
    std::vector<std::pair<int, double>> missed_track_probability;
    int hypothesis_count = 0;
};

struct mht_branch {
    association_hypothesis hypothesis;
    double score = 0.0;
    int depth = 0;
};

namespace detail {

inline void enumerate_association_hypotheses(const std::vector<track_state>& tracks,
                                             const std::vector<detection>& detections,
                                             double gate_threshold,
                                             double miss_cost,
                                             std::size_t track_index,
                                             std::vector<bool>& det_used,
                                             association_hypothesis& current,
                                             std::vector<association_hypothesis>& out) {
    if (track_index >= tracks.size()) {
        out.push_back(current);
        return;
    }

    const track_state& track = tracks[track_index];

    current.assignments.push_back({track.id, -1, miss_cost});
    current.total_cost += miss_cost;
    enumerate_association_hypotheses(tracks, detections, gate_threshold, miss_cost, track_index + 1,
                                     det_used, current, out);
    current.total_cost -= miss_cost;
    current.assignments.pop_back();

    for (std::size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
        if (det_used[det_idx]) continue;
        double cost = nearest_neighbor_associator::mahalanobis_distance(track, detections[det_idx]);
        if (cost > gate_threshold) continue;

        det_used[det_idx] = true;
        current.assignments.push_back({track.id, static_cast<int>(det_idx), cost});
        current.total_cost += cost;
        enumerate_association_hypotheses(tracks, detections, gate_threshold, miss_cost, track_index + 1,
                                         det_used, current, out);
        current.total_cost -= cost;
        current.assignments.pop_back();
        det_used[det_idx] = false;
    }
}

inline std::vector<association_hypothesis> build_hypotheses(const std::vector<track_state>& tracks,
                                                            const std::vector<detection>& detections,
                                                            double gate_threshold,
                                                            double miss_cost) {
    std::vector<association_hypothesis> out;
    std::vector<bool> det_used(detections.size(), false);
    association_hypothesis seed;
    enumerate_association_hypotheses(tracks, detections, gate_threshold, miss_cost, 0, det_used, seed, out);
    return out;
}

}  // namespace detail

struct gnn_associator {
    double gate_threshold = 16.0;
    double miss_cost = 8.0;

    association_hypothesis associate(const std::vector<track_state>& tracks,
                                     const std::vector<detection>& detections) const {
        association_hypothesis best;
        best.total_cost = std::numeric_limits<double>::infinity();
        auto hypotheses = detail::build_hypotheses(tracks, detections, gate_threshold, miss_cost);
        for (const auto& h : hypotheses) {
            if (h.total_cost < best.total_cost) best = h;
        }
        if (!std::isfinite(best.total_cost)) best.total_cost = 0.0;
        return best;
    }
};

struct jpda_associator {
    double gate_threshold = 16.0;
    double miss_cost = 8.0;

    jpda_result associate(const std::vector<track_state>& tracks,
                          const std::vector<detection>& detections) const {
        jpda_result out;
        auto hypotheses = detail::build_hypotheses(tracks, detections, gate_threshold, miss_cost);
        out.hypothesis_count = static_cast<int>(hypotheses.size());
        if (hypotheses.empty()) return out;

        std::vector<double> weights(hypotheses.size(), 0.0);
        double total_weight = 0.0;
        for (std::size_t i = 0; i < hypotheses.size(); ++i) {
            weights[i] = std::exp(-0.5 * hypotheses[i].total_cost);
            total_weight += weights[i];
        }
        total_weight = std::max(total_weight, 1.0e-20);

        for (std::size_t h = 0; h < hypotheses.size(); ++h) {
            double prob = weights[h] / total_weight;
            for (const auto& a : hypotheses[h].assignments) {
                if (a.detection_idx >= 0) {
                    auto it = std::find_if(out.marginals.begin(), out.marginals.end(),
                                           [&](const jpda_pair_probability& v) {
                                               return v.track_id == a.track_id &&
                                                      v.detection_idx == a.detection_idx;
                                           });
                    if (it == out.marginals.end()) {
                        out.marginals.push_back({a.track_id, a.detection_idx, prob});
                    } else {
                        it->probability += prob;
                    }
                } else {
                    auto it = std::find_if(out.missed_track_probability.begin(),
                                           out.missed_track_probability.end(),
                                           [&](const std::pair<int, double>& v) {
                                               return v.first == a.track_id;
                                           });
                    if (it == out.missed_track_probability.end()) {
                        out.missed_track_probability.push_back({a.track_id, prob});
                    } else {
                        it->second += prob;
                    }
                }
            }
        }
        return out;
    }
};

struct mht_associator {
    double gate_threshold = 16.0;
    double miss_cost = 8.0;
    int max_branches = 4;

    std::vector<mht_branch> expand(const std::vector<track_state>& tracks,
                                   const std::vector<detection>& detections,
                                   int depth = 1) const {
        std::vector<mht_branch> out;
        auto hypotheses = detail::build_hypotheses(tracks, detections, gate_threshold, miss_cost);
        for (const auto& h : hypotheses) {
            out.push_back({h, std::exp(-0.5 * h.total_cost), depth});
        }
        std::sort(out.begin(), out.end(), [](const mht_branch& a, const mht_branch& b) {
            return a.score > b.score;
        });
        if (static_cast<int>(out.size()) > max_branches) out.resize(static_cast<std::size_t>(max_branches));
        return out;
    }
};

}  // namespace xsf_math
