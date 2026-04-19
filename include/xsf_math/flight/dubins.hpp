#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

// Dubins 路径位姿（Dubins Path Pose）
struct dubins_pose {
    double x_m = 0.0;  // X 坐标，单位米（X Coordinate in Meters）
    double y_m = 0.0;  // Y 坐标，单位米（Y Coordinate in Meters）
    double heading_rad = 0.0;  // 航向角，单位弧度（Heading Angle in Radians）
};

// Dubins 路径段类型（Dubins Segment Kind）
enum class dubins_segment_kind {
    left,     // 左转弯（Left Turn）
    straight, // 直线（Straight）
    right     // 右转弯（Right Turn）
};

// Dubins 路径段（Dubins Path Segment）
struct dubins_segment {
    dubins_segment_kind kind = dubins_segment_kind::straight;  // 段类型（Segment Kind）
    double length_m = 0.0;  // 段长度，单位米（Segment Length in Meters）
    double turn_radius_m = 0.0;  // 转弯半径，单位米（Turn Radius in Meters）
};

// 圆形障碍物（Circular Obstacle）
struct circular_obstacle {
    vec3 center_wcs{};  // 中心点世界坐标（Obstacle Center in WCS）
    double radius_m = 0.0;  // 半径，单位米（Radius in Meters）
};

// Dubins 路径结果（Dubins Path Result）
struct dubins_path {
    bool valid = false;  // 路径是否有效（Whether Path is Valid）
    std::vector<dubins_segment> segments;  // 路径段序列（Sequence of Path Segments）
    std::vector<vec3> waypoints_wcs;  // 路径航点世界坐标（Path Waypoints in WCS）
    double total_length_m = 0.0;  // 总长度，单位米（Total Length in Meters）
};

namespace detail {

// 将航向角归一化到 [-pi, pi]（Wrap Heading Angle to [-pi, pi]）
inline double wrap_heading(double rad) {
    while (rad > constants::pi) rad -= constants::two_pi;
    while (rad < -constants::pi) rad += constants::two_pi;
    return rad;
}

// 点到线段距离的平方（Squared Point-to-Segment Distance）
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

// Dubins 路径规划器（Dubins Path Planner）
struct dubins_planner {
    double minimum_turn_radius_m = 300.0;  // 最小转弯半径，单位米（Minimum Turn Radius in Meters）

    // 规划从起点到目标点的 Dubins 路径（Plan Dubins Path from Start to Goal）
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
