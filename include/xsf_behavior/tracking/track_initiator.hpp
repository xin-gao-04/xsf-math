#pragma once

#include <xsf_behavior/tracking/track_manager.hpp>
#include <array>
#include <vector>

namespace xsf_math {

struct detection_sample {
    detection measurement{};
    double sim_time_s = 0.0;
};

struct track_seed {
    detection first_detection{};
    vec3 initial_velocity{};
    bool valid = false;
};

struct track_initiator {
    track_seed seed_from_two_points(const detection_sample& a, const detection_sample& b) const {
        track_seed seed;
        double dt = b.sim_time_s - a.sim_time_s;
        if (dt <= 1.0e-6) return seed;
        seed.first_detection = b.measurement;
        seed.initial_velocity = (b.measurement.position - a.measurement.position) / dt;
        seed.valid = true;
        return seed;
    }

    track_seed seed_from_three_points(const detection_sample& a,
                                      const detection_sample& b,
                                      const detection_sample& c) const {
        track_seed seed;
        double dt1 = b.sim_time_s - a.sim_time_s;
        double dt2 = c.sim_time_s - b.sim_time_s;
        if (dt1 <= 1.0e-6 || dt2 <= 1.0e-6) return seed;
        vec3 v1 = (b.measurement.position - a.measurement.position) / dt1;
        vec3 v2 = (c.measurement.position - b.measurement.position) / dt2;
        seed.first_detection = c.measurement;
        seed.initial_velocity = 0.5 * (v1 + v2);
        seed.valid = true;
        return seed;
    }

    int start_track(track_manager& manager, const track_seed& seed, double sim_time_s) const {
        if (!seed.valid) return -1;
        return manager.start_tentative_track(seed.first_detection, sim_time_s, seed.initial_velocity);
    }
};

}  // namespace xsf_math
