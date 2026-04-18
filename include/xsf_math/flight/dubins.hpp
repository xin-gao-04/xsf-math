#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

struct dubins_pose {
    double x_m = 0.0;
    double y_m = 0.0;
    double heading_rad = 0.0;
};

enum class dubins_segment_kind { left, straight, right };

struct dubins_segment {
    dubins_segment_kind kind = dubins_segment_kind::straight;
    double length_m = 0.0;
    double turn_radius_m = 0.0;
};

struct circular_obstacle {
    vec3 center_wcs{};
    double radius_m = 0.0;
};

struct dubins_path {
    bool valid = false;
    std::vector<dubins_segment> segments;
    std::vector<vec3> waypoints_wcs;
    double total_length_m = 0.0;
};

namespace detail {

inline double wrap_heading(double rad) {
    while (rad > constants::pi) rad -= constants::two_pi;
    while (rad < -constants::pi) rad += constants::two_pi;
    return rad;
}

inline double point_to_segment_distance_sq(double px, double py,
                                           double ax, double ay,
                                           double bx, double by) {
    double abx = bx - ax;
    double aby = by - ay;
    double apx = px - ax;
    double apy = py - ay;
    double ab2 = abx * abx + aby * aby;
    double t = (ab2 > 1.0e-12) ? std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0) : 0.0;
    double cx = ax + t * abx;
    double cy = ay + t * aby;
    double dx = px - cx;
    double dy = py - cy;
    return dx * dx + dy * dy;
}

}  // namespace detail

struct dubins_planner {
    double minimum_turn_radius_m = 300.0;

    dubins_path plan(const dubins_pose& start,
                     const dubins_pose& goal,
                     const std::vector<circular_obstacle>& obstacles = {}) const {
        dubins_path out;
        vec3 start_pt{start.x_m, start.y_m, 0.0};
        vec3 goal_pt{goal.x_m, goal.y_m, 0.0};
        out.waypoints_wcs.push_back(start_pt);

        vec3 delta = goal_pt - start_pt;
        double straight = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (straight < 1.0e-6) return out;

        bool need_detour = false;
        vec3 detour{};
        for (const auto& obs : obstacles) {
            double d2 = detail::point_to_segment_distance_sq(obs.center_wcs.x, obs.center_wcs.y,
                                                             start.x_m, start.y_m, goal.x_m, goal.y_m);
            double safe_r = obs.radius_m + minimum_turn_radius_m;
            if (d2 < safe_r * safe_r) {
                need_detour = true;
                vec3 dir = delta.normalized();
                vec3 normal{-dir.y, dir.x, 0.0};
                double side = ((obs.center_wcs - start_pt).dot(normal) >= 0.0) ? -1.0 : 1.0;
                detour = obs.center_wcs + normal * (side * safe_r);
                break;
            }
        }

        double start_turn = std::abs(detail::wrap_heading(std::atan2(delta.y, delta.x) - start.heading_rad));
        double end_turn = std::abs(detail::wrap_heading(goal.heading_rad - std::atan2(delta.y, delta.x)));
        out.segments.push_back({detail::wrap_heading(std::atan2(delta.y, delta.x) - start.heading_rad) >= 0.0
                                    ? dubins_segment_kind::left
                                    : dubins_segment_kind::right,
                                minimum_turn_radius_m * start_turn,
                                minimum_turn_radius_m});

        if (need_detour) {
            out.waypoints_wcs.push_back(detour);
            out.segments.push_back({dubins_segment_kind::straight, (detour - start_pt).magnitude(), minimum_turn_radius_m});
            out.segments.push_back({dubins_segment_kind::straight, (goal_pt - detour).magnitude(), minimum_turn_radius_m});
            out.total_length_m = out.segments[0].length_m + out.segments[1].length_m + out.segments[2].length_m;
        } else {
            out.segments.push_back({dubins_segment_kind::straight, straight, minimum_turn_radius_m});
            out.total_length_m = out.segments[0].length_m + out.segments[1].length_m;
        }

        out.segments.push_back({detail::wrap_heading(goal.heading_rad - std::atan2(delta.y, delta.x)) >= 0.0
                                    ? dubins_segment_kind::left
                                    : dubins_segment_kind::right,
                                minimum_turn_radius_m * end_turn,
                                minimum_turn_radius_m});
        out.total_length_m += out.segments.back().length_m;
        out.waypoints_wcs.push_back(goal_pt);
        out.valid = true;
        return out;
    }
};

}  // namespace xsf_math
