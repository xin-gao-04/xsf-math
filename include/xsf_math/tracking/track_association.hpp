#pragma once

#include "../core/vec3.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

namespace xsf_math {

// Track state for association
struct track_state {
    int    id = -1;
    vec3   position;
    vec3   velocity;
    double position_covariance[3] = {100.0, 100.0, 100.0};  // diagonal elements
};

// Detection measurement
struct detection {
    vec3   position;
    double meas_noise[3] = {100.0, 100.0, 100.0};
};

// Association result
struct association_result {
    int track_id    = -1;   // -1 means unassociated
    int detection_idx = -1;
    double distance = 0.0;  // Mahalanobis or Euclidean distance
};

// Nearest-neighbor association with gating
struct nearest_neighbor_associator {

    double gate_threshold = 16.0;  // Chi-squared gate (3 DOF, ~99.7% for 16.0)

    // Compute Mahalanobis distance between track predicted position and detection
    static double mahalanobis_distance(const track_state& trk,
                                       const detection& det) {
        double dx = det.position.x - trk.position.x;
        double dy = det.position.y - trk.position.y;
        double dz = det.position.z - trk.position.z;

        // Combined covariance = track_cov + meas_noise (diagonal)
        double sx = trk.position_covariance[0] + det.meas_noise[0];
        double sy = trk.position_covariance[1] + det.meas_noise[1];
        double sz = trk.position_covariance[2] + det.meas_noise[2];

        if (sx < 1e-20) sx = 1e-20;
        if (sy < 1e-20) sy = 1e-20;
        if (sz < 1e-20) sz = 1e-20;

        return (dx*dx/sx) + (dy*dy/sy) + (dz*dz/sz);
    }

    // Associate detections to tracks (greedy nearest-neighbor)
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

        // Report unassociated detections
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

// M-of-N track confirmation logic
struct m_of_n_logic {
    int m = 3;  // required detections
    int n = 5;  // window size

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
        // Simple sliding window approximation
        while (s.total > n) {
            // Assume oldest entry was proportionally hit
            double hit_rate = (s.total > 0) ? static_cast<double>(s.hits) / s.total : 0.0;
            if (hit_rate > 0.5 && s.hits > 0) s.hits--;
            s.total--;
        }
    }
};

} // namespace xsf_math
