#pragma once

#include "vec3.hpp"
#include "mat3.hpp"
#include "constants.hpp"
#include <cmath>

namespace xsf_math {

// 地理坐标
struct lla {
    double lat_rad = 0.0;  // 纬度（弧度）
    double lon_rad = 0.0;  // 经度（弧度）
    double alt_m   = 0.0;  // 高度（米，基于 WGS-84 椭球或海平面）
};

// 欧拉角（航空航天约定：yaw-pitch-roll / heading-pitch-roll）
struct euler_angles {
    double heading_rad = 0.0;  // psi   - 绕 Z（向下）轴旋转，0=北，顺时针为正
    double pitch_rad   = 0.0;  // theta - 绕 Y（右）轴旋转，机头上仰为正
    double roll_rad    = 0.0;  // phi   - 绕 X（前向）轴旋转，右机翼下垂为正
};

// 由欧拉角生成方向余弦矩阵
// 将 WCS（北-东-下）转换到 ECS（前-右-下）
inline mat3 dcm_wcs_to_ecs(const euler_angles& e) {
    double ch = std::cos(e.heading_rad), sh = std::sin(e.heading_rad);
    double cp = std::cos(e.pitch_rad),   sp = std::sin(e.pitch_rad);
    double cr = std::cos(e.roll_rad),    sr = std::sin(e.roll_rad);

    mat3 r;
    // 第 0 行：WCS 中的前向轴
    r.m[0][0] = ch * cp;
    r.m[0][1] = sh * cp;
    r.m[0][2] = -sp;
    // 第 1 行：WCS 中的右向轴
    r.m[1][0] = ch * sp * sr - sh * cr;
    r.m[1][1] = sh * sp * sr + ch * cr;
    r.m[1][2] = cp * sr;
    // 第 2 行：WCS 中的下向轴
    r.m[2][0] = ch * sp * cr + sh * sr;
    r.m[2][1] = sh * sp * cr - ch * sr;
    r.m[2][2] = cp * cr;
    return r;
}

// 逆变换：ECS -> WCS（由于 DCM 正交，取转置即可）
inline mat3 dcm_ecs_to_wcs(const euler_angles& e) {
    return dcm_wcs_to_ecs(e).transposed();
}

// 将 WCS 向量转换为 ECS
inline vec3 wcs_to_ecs(const vec3& v_wcs, const euler_angles& attitude) {
    return dcm_wcs_to_ecs(attitude) * v_wcs;
}

// 将 ECS 向量转换为 WCS
inline vec3 ecs_to_wcs(const vec3& v_ecs, const euler_angles& attitude) {
    return dcm_ecs_to_wcs(attitude) * v_ecs;
}

// 由 WCS（NED）向量求方位角和俯仰角
// 方位角：从北向（X）起算，顺时针为正 [0, 2pi)
// 俯仰角：相对地平线向上为正
inline double azimuth_from_vec(const vec3& v) {
    double az = std::atan2(v.y, v.x);
    if (az < 0.0) az += constants::two_pi;
    return az;
}

inline double elevation_from_vec(const vec3& v) {
    double range = v.magnitude();
    if (range < 1e-30) return 0.0;
    return -std::asin(v.z / range);  // Z 向下，因此对“向上”俯仰取负号
}

// 由方位角/俯仰角构造单位方向向量（WCS NED）
inline vec3 vec_from_az_el(double az_rad, double el_rad) {
    double ce = std::cos(el_rad);
    return { ce * std::cos(az_rad), ce * std::sin(az_rad), -std::sin(el_rad) };
}

// 天线坐标系（ACS）变换
// 已知平台上的天线安装角（az_mount, el_mount）
// 以及平台姿态，计算从 WCS 到 ACS 的 DCM
inline mat3 dcm_wcs_to_acs(const euler_angles& platform_attitude,
                            double ant_az_mount_rad,
                            double ant_el_mount_rad) {
    // 平台 WCS->ECS
    mat3 wcs_to_ecs_mat = dcm_wcs_to_ecs(platform_attitude);

    // 天线安装旋转（在 ECS 中先方位后俯仰）
    double ca = std::cos(ant_az_mount_rad), sa = std::sin(ant_az_mount_rad);
    double ce = std::cos(ant_el_mount_rad), se = std::sin(ant_el_mount_rad);

    mat3 ant_mount;
    ant_mount.m[0][0] =  ca * ce;  ant_mount.m[0][1] = sa * ce;  ant_mount.m[0][2] = -se;
    ant_mount.m[1][0] = -sa;       ant_mount.m[1][1] = ca;        ant_mount.m[1][2] =  0.0;
    ant_mount.m[2][0] =  ca * se;  ant_mount.m[2][1] = sa * se;   ant_mount.m[2][2] =  ce;

    return ant_mount * wcs_to_ecs_mat;
}

// 两个 LLA 点之间的大圆距离（Haversine 公式）
inline double great_circle_distance(const lla& a, const lla& b) {
    double dlat = b.lat_rad - a.lat_rad;
    double dlon = b.lon_rad - a.lon_rad;
    double h = std::sin(dlat/2) * std::sin(dlat/2)
             + std::cos(a.lat_rad) * std::cos(b.lat_rad) * std::sin(dlon/2) * std::sin(dlon/2);
    return 2.0 * constants::earth_radius_m * std::asin(std::sqrt(h));
}

// 平地近似：将 LLA 差值转换为局部 NED 偏移（米）
// 参考点为 `ref`
inline vec3 lla_to_local_ned(const lla& ref, const lla& point) {
    double dlat = point.lat_rad - ref.lat_rad;
    double dlon = point.lon_rad - ref.lon_rad;
    double north = dlat * constants::earth_radius_m;
    double east  = dlon * constants::earth_radius_m * std::cos(ref.lat_rad);
    double down  = -(point.alt_m - ref.alt_m);
    return {north, east, down};
}

} // namespace xsf_math
