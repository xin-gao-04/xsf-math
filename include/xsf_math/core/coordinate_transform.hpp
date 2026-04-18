#pragma once

#include "constants.hpp"
#include "mat3.hpp"
#include "quaternion.hpp"
#include "vec3.hpp"
#include <cmath>

namespace xsf_math {

// 地理坐标
struct lla {
    double lat_rad = 0.0;  // 纬度（弧度）
    double lon_rad = 0.0;  // 经度（弧度）
    double alt_m   = 0.0;  // 高度（米，基于 WGS-84 椭球）
};

// 欧拉角（航空航天约定：yaw-pitch-roll / heading-pitch-roll）
struct euler_angles {
    double heading_rad = 0.0;  // psi   - 绕 Z（向下）轴旋转，0=北，顺时针为正
    double pitch_rad   = 0.0;  // theta - 绕 Y（右）轴旋转，机头上仰为正
    double roll_rad    = 0.0;  // phi   - 绕 X（前向）轴旋转，右机翼下垂为正
};

inline double wrap_lon_rad(double lon_rad) {
    double wrapped = std::fmod(lon_rad + constants::pi, constants::two_pi);
    if (wrapped < 0.0) wrapped += constants::two_pi;
    return wrapped - constants::pi;
}

// 由欧拉角生成方向余弦矩阵
// 将 WCS（北-东-下）转换到 ECS（前-右-下）
inline mat3 dcm_wcs_to_ecs(const euler_angles& e) {
    double ch = std::cos(e.heading_rad), sh = std::sin(e.heading_rad);
    double cp = std::cos(e.pitch_rad),   sp = std::sin(e.pitch_rad);
    double cr = std::cos(e.roll_rad),    sr = std::sin(e.roll_rad);

    mat3 r;
    r.m[0][0] = ch * cp;
    r.m[0][1] = sh * cp;
    r.m[0][2] = -sp;
    r.m[1][0] = ch * sp * sr - sh * cr;
    r.m[1][1] = sh * sp * sr + ch * cr;
    r.m[1][2] = cp * sr;
    r.m[2][0] = ch * sp * cr + sh * sr;
    r.m[2][1] = sh * sp * cr - ch * sr;
    r.m[2][2] = cp * cr;
    return r;
}

inline mat3 dcm_ecs_to_wcs(const euler_angles& e) { return dcm_wcs_to_ecs(e).transposed(); }

inline quaternion quaternion_wcs_to_ecs(const euler_angles& e) {
    return quaternion::from_dcm(dcm_wcs_to_ecs(e));
}

inline euler_angles quaternion_to_euler(const quaternion& q_in) {
    quaternion q = q_in.normalized();
    mat3 dcm = q.to_dcm();
    euler_angles e;
    e.pitch_rad = -std::asin(std::clamp(dcm.m[0][2], -1.0, 1.0));
    e.heading_rad = std::atan2(dcm.m[0][1], dcm.m[0][0]);
    e.roll_rad = std::atan2(dcm.m[1][2], dcm.m[2][2]);
    return e;
}

inline vec3 wcs_to_ecs(const vec3& v_wcs, const euler_angles& attitude) {
    return dcm_wcs_to_ecs(attitude) * v_wcs;
}

inline vec3 ecs_to_wcs(const vec3& v_ecs, const euler_angles& attitude) {
    return dcm_ecs_to_wcs(attitude) * v_ecs;
}

inline vec3 wcs_to_ecs(const vec3& v_wcs, const quaternion& q_wcs_to_ecs) {
    return q_wcs_to_ecs.normalized().rotate(v_wcs);
}

inline vec3 ecs_to_wcs(const vec3& v_ecs, const quaternion& q_wcs_to_ecs) {
    return q_wcs_to_ecs.normalized().conjugate().rotate(v_ecs);
}

// 由 WCS（NED）向量求方位角和俯仰角
inline double azimuth_from_vec(const vec3& v) {
    double az = std::atan2(v.y, v.x);
    if (az < 0.0) az += constants::two_pi;
    return az;
}

inline double elevation_from_vec(const vec3& v) {
    double range = v.magnitude();
    if (range < 1e-30) return 0.0;
    return -std::asin(v.z / range);
}

inline vec3 vec_from_az_el(double az_rad, double el_rad) {
    double ce = std::cos(el_rad);
    return {ce * std::cos(az_rad), ce * std::sin(az_rad), -std::sin(el_rad)};
}

inline mat3 dcm_wcs_to_acs(const euler_angles& platform_attitude,
                            double ant_az_mount_rad,
                            double ant_el_mount_rad) {
    mat3 wcs_to_ecs_mat = dcm_wcs_to_ecs(platform_attitude);
    double ca = std::cos(ant_az_mount_rad), sa = std::sin(ant_az_mount_rad);
    double ce = std::cos(ant_el_mount_rad), se = std::sin(ant_el_mount_rad);

    mat3 ant_mount;
    ant_mount.m[0][0] =  ca * ce;  ant_mount.m[0][1] = sa * ce;  ant_mount.m[0][2] = -se;
    ant_mount.m[1][0] = -sa;       ant_mount.m[1][1] = ca;       ant_mount.m[1][2] =  0.0;
    ant_mount.m[2][0] =  ca * se;  ant_mount.m[2][1] = sa * se;  ant_mount.m[2][2] =  ce;
    return ant_mount * wcs_to_ecs_mat;
}

inline vec3 lla_to_ecef(const lla& geo) {
    double a = constants::earth_equatorial_radius_m;
    double e2 = constants::earth_eccentricity_sq;
    double sin_lat = std::sin(geo.lat_rad);
    double cos_lat = std::cos(geo.lat_rad);
    double sin_lon = std::sin(geo.lon_rad);
    double cos_lon = std::cos(geo.lon_rad);
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    return {
        (N + geo.alt_m) * cos_lat * cos_lon,
        (N + geo.alt_m) * cos_lat * sin_lon,
        (N * (1.0 - e2) + geo.alt_m) * sin_lat
    };
}

inline lla ecef_to_lla(const vec3& ecef) {
    double a = constants::earth_equatorial_radius_m;
    double e2 = constants::earth_eccentricity_sq;
    double b = a * (1.0 - constants::earth_flattening);
    double ep2 = (a * a - b * b) / (b * b);
    double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double th = std::atan2(a * ecef.z, b * p);
    double sin_th = std::sin(th), cos_th = std::cos(th);

    lla geo;
    geo.lon_rad = wrap_lon_rad(std::atan2(ecef.y, ecef.x));
    geo.lat_rad = std::atan2(ecef.z + ep2 * b * sin_th * sin_th * sin_th,
                             p - e2 * a * cos_th * cos_th * cos_th);

    double sin_lat = std::sin(geo.lat_rad);
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    geo.alt_m = p / std::cos(geo.lat_rad) - N;
    return geo;
}

inline mat3 dcm_ecef_to_ned(const lla& ref) {
    double slat = std::sin(ref.lat_rad), clat = std::cos(ref.lat_rad);
    double slon = std::sin(ref.lon_rad), clon = std::cos(ref.lon_rad);
    return mat3::from_rows(
        {-slat * clon, -slat * slon, clat},
        {-slon,         clon,        0.0},
        {-clat * clon, -clat * slon, -slat}
    );
}

inline mat3 dcm_ned_to_ecef(const lla& ref) { return dcm_ecef_to_ned(ref).transposed(); }

inline vec3 ecef_to_local_ned(const lla& ref, const vec3& point_ecef) {
    vec3 ref_ecef = lla_to_ecef(ref);
    return dcm_ecef_to_ned(ref) * (point_ecef - ref_ecef);
}

inline vec3 local_ned_to_ecef(const lla& ref, const vec3& ned) {
    vec3 ref_ecef = lla_to_ecef(ref);
    return ref_ecef + dcm_ned_to_ecef(ref) * ned;
}

inline double great_circle_distance(const lla& a, const lla& b) {
    double dlat = b.lat_rad - a.lat_rad;
    double dlon = b.lon_rad - a.lon_rad;
    double h = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(a.lat_rad) * std::cos(b.lat_rad) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    return 2.0 * constants::earth_radius_m * std::asin(std::sqrt(h));
}

inline vec3 lla_to_local_ned(const lla& ref, const lla& point) {
    return ecef_to_local_ned(ref, lla_to_ecef(point));
}

inline double gmst_from_julian_date(double julian_date) {
    double T = (julian_date - constants::julian_date_j2000) / 36525.0;
    double gmst_deg = 280.46061837 +
                      360.98564736629 * (julian_date - constants::julian_date_j2000) +
                      0.000387933 * T * T -
                      T * T * T / 38710000.0;
    return wrap_lon_rad(gmst_deg * constants::deg_to_rad);
}

inline vec3 eci_to_ecef(const vec3& eci, double julian_date) {
    double theta = gmst_from_julian_date(julian_date);
    double c = std::cos(theta), s = std::sin(theta);
    return {c * eci.x + s * eci.y, -s * eci.x + c * eci.y, eci.z};
}

inline vec3 ecef_to_eci(const vec3& ecef, double julian_date) {
    double theta = gmst_from_julian_date(julian_date);
    double c = std::cos(theta), s = std::sin(theta);
    return {c * ecef.x - s * ecef.y, s * ecef.x + c * ecef.y, ecef.z};
}

inline vec3 eci_to_local_ned(const vec3& eci, const lla& ref, double julian_date) {
    return ecef_to_local_ned(ref, eci_to_ecef(eci, julian_date));
}

inline vec3 local_ned_to_eci(const vec3& ned, const lla& ref, double julian_date) {
    return ecef_to_eci(local_ned_to_ecef(ref, ned), julian_date);
}

}  // namespace xsf_math
