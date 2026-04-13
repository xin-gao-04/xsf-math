#pragma once

#include "../core/constants.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

namespace xsf_math {

// Probability of Kill (Pk) models

// Pk vs miss distance (graduated lethality curve)
struct pk_curve {
    std::vector<double> miss_distance_m;  // sorted ascending
    std::vector<double> pk_values;        // corresponding Pk [0,1]

    // Evaluate Pk for given miss distance
    double evaluate(double miss_dist_m) const {
        if (miss_distance_m.empty()) return 0.0;
        return table_lookup(miss_distance_m, pk_values, std::abs(miss_dist_m));
    }

    // Create typical blast-fragmentation warhead curve
    static pk_curve blast_fragmentation(double lethal_radius_m, double max_radius_m) {
        pk_curve curve;
        int n = 20;
        for (int i = 0; i <= n; ++i) {
            double d = max_radius_m * static_cast<double>(i) / n;
            curve.miss_distance_m.push_back(d);

            double pk;
            if (d <= lethal_radius_m * 0.5) {
                pk = 1.0;  // certain kill within half lethal radius
            } else if (d <= lethal_radius_m) {
                // Smooth falloff
                double t = (d - lethal_radius_m * 0.5) / (lethal_radius_m * 0.5);
                pk = 1.0 - 0.3 * t * t;
            } else {
                // Exponential decay beyond lethal radius
                double excess = (d - lethal_radius_m) / lethal_radius_m;
                pk = 0.7 * std::exp(-2.0 * excess * excess);
            }
            curve.pk_values.push_back(std::max(pk, 0.0));
        }
        return curve;
    }

    // Create continuous rod warhead curve
    static pk_curve continuous_rod(double rod_radius_m) {
        pk_curve curve;
        curve.miss_distance_m = {0.0, rod_radius_m * 0.5, rod_radius_m, rod_radius_m * 1.5, rod_radius_m * 3.0};
        curve.pk_values       = {1.0, 0.95,                0.6,          0.1,                  0.0};
        return curve;
    }
};

// Kill assessment using miss distance squared (avoids sqrt)
struct kill_assessment {
    double lethal_radius_sq_m2;  // lethal radius squared

    // Quick check: is miss distance within lethal envelope?
    bool within_lethal_envelope(double miss_dist_sq_m2) const {
        return miss_dist_sq_m2 <= lethal_radius_sq_m2;
    }
};

// Single-shot Pk (SSPK) computation
inline double single_shot_pk(double miss_distance_m,
                              const pk_curve& curve) {
    return curve.evaluate(miss_distance_m);
}

// Cumulative Pk for N independent shots
inline double cumulative_pk(const std::vector<double>& individual_pks) {
    double p_survive = 1.0;
    for (double pk : individual_pks) {
        p_survive *= (1.0 - pk);
    }
    return 1.0 - p_survive;
}

// Monte Carlo kill determination
struct monte_carlo_kill {
    std::mt19937 rng;

    explicit monte_carlo_kill(unsigned seed = 42) : rng(seed) {}

    // Returns true if target is killed
    bool evaluate(double pk) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(rng) < pk;
    }

    // Evaluate with miss distance and Pk curve
    bool evaluate(double miss_distance_m, const pk_curve& curve) {
        double pk = curve.evaluate(miss_distance_m);
        return evaluate(pk);
    }
};

// Target vulnerability classification
enum class target_class {
    fighter,
    bomber,
    transport,
    cruise_missile,
    ballistic_missile,
    helicopter,
    uav,
    ground_vehicle,
    ship
};

// Typical lethal radius by target class (meters)
inline double typical_lethal_radius(target_class tc) {
    switch (tc) {
    case target_class::fighter:           return 8.0;
    case target_class::bomber:            return 12.0;
    case target_class::transport:         return 15.0;
    case target_class::cruise_missile:    return 5.0;
    case target_class::ballistic_missile: return 10.0;
    case target_class::helicopter:        return 10.0;
    case target_class::uav:              return 6.0;
    case target_class::ground_vehicle:    return 3.0;
    case target_class::ship:              return 20.0;
    default: return 10.0;
    }
}

// EW degradation of Pk
// Jamming can reduce tracking accuracy -> increase miss distance
inline double ew_degraded_miss_distance(double baseline_miss_m,
                                         double track_error_increase_m) {
    return std::sqrt(baseline_miss_m * baseline_miss_m +
                     track_error_increase_m * track_error_increase_m);
}

} // namespace xsf_math
