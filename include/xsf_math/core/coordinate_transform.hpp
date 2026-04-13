#pragma once

#include "vec3.hpp"
#include "mat3.hpp"
#include "constants.hpp"
#include <cmath>

namespace xsf_math {

// Geographic coordinates
struct lla {
    double lat_rad = 0.0;  // latitude in radians
    double lon_rad = 0.0;  // longitude in radians
    double alt_m   = 0.0;  // altitude in meters (above WGS-84 ellipsoid or MSL)
};

// Euler angles (aerospace convention: yaw-pitch-roll / heading-pitch-roll)
struct euler_angles {
    double heading_rad = 0.0;  // psi   - rotation about Z (down) axis, 0=North, CW positive
    double pitch_rad   = 0.0;  // theta - rotation about Y (right) axis, nose up positive
    double roll_rad    = 0.0;  // phi   - rotation about X (forward) axis, right wing down positive
};

// Direction Cosine Matrix from Euler angles
// Transforms WCS (North-East-Down) to ECS (Forward-Right-Down)
inline mat3 dcm_wcs_to_ecs(const euler_angles& e) {
    double ch = std::cos(e.heading_rad), sh = std::sin(e.heading_rad);
    double cp = std::cos(e.pitch_rad),   sp = std::sin(e.pitch_rad);
    double cr = std::cos(e.roll_rad),    sr = std::sin(e.roll_rad);

    mat3 r;
    // Row 0: forward axis in WCS
    r.m[0][0] = ch * cp;
    r.m[0][1] = sh * cp;
    r.m[0][2] = -sp;
    // Row 1: right axis in WCS
    r.m[1][0] = ch * sp * sr - sh * cr;
    r.m[1][1] = sh * sp * sr + ch * cr;
    r.m[1][2] = cp * sr;
    // Row 2: down axis in WCS
    r.m[2][0] = ch * sp * cr + sh * sr;
    r.m[2][1] = sh * sp * cr - ch * sr;
    r.m[2][2] = cp * cr;
    return r;
}

// Inverse: ECS to WCS (transpose since DCM is orthogonal)
inline mat3 dcm_ecs_to_wcs(const euler_angles& e) {
    return dcm_wcs_to_ecs(e).transposed();
}

// Convert WCS vector to ECS
inline vec3 wcs_to_ecs(const vec3& v_wcs, const euler_angles& attitude) {
    return dcm_wcs_to_ecs(attitude) * v_wcs;
}

// Convert ECS vector to WCS
inline vec3 ecs_to_wcs(const vec3& v_ecs, const euler_angles& attitude) {
    return dcm_ecs_to_wcs(attitude) * v_ecs;
}

// Azimuth and elevation from a vector in WCS (NED)
// Azimuth: from North (X), clockwise positive [0, 2pi)
// Elevation: above horizon, positive up
inline double azimuth_from_vec(const vec3& v) {
    double az = std::atan2(v.y, v.x);
    if (az < 0.0) az += constants::two_pi;
    return az;
}

inline double elevation_from_vec(const vec3& v) {
    double range = v.magnitude();
    if (range < 1e-30) return 0.0;
    return -std::asin(v.z / range);  // Z is down, so negate for "up" elevation
}

// Construct unit direction vector from azimuth/elevation (WCS NED)
inline vec3 vec_from_az_el(double az_rad, double el_rad) {
    double ce = std::cos(el_rad);
    return { ce * std::cos(az_rad), ce * std::sin(az_rad), -std::sin(el_rad) };
}

// Antenna Coordinate System (ACS) transform
// Given antenna mounting angles on the platform (az_mount, el_mount)
// and the platform attitude, compute DCM from WCS to ACS
inline mat3 dcm_wcs_to_acs(const euler_angles& platform_attitude,
                            double ant_az_mount_rad,
                            double ant_el_mount_rad) {
    // Platform WCS->ECS
    mat3 wcs_to_ecs_mat = dcm_wcs_to_ecs(platform_attitude);

    // Antenna mounting rotation (az then el in ECS)
    double ca = std::cos(ant_az_mount_rad), sa = std::sin(ant_az_mount_rad);
    double ce = std::cos(ant_el_mount_rad), se = std::sin(ant_el_mount_rad);

    mat3 ant_mount;
    ant_mount.m[0][0] =  ca * ce;  ant_mount.m[0][1] = sa * ce;  ant_mount.m[0][2] = -se;
    ant_mount.m[1][0] = -sa;       ant_mount.m[1][1] = ca;        ant_mount.m[1][2] =  0.0;
    ant_mount.m[2][0] =  ca * se;  ant_mount.m[2][1] = sa * se;   ant_mount.m[2][2] =  ce;

    return ant_mount * wcs_to_ecs_mat;
}

// Great circle distance between two LLA points (Haversine formula)
inline double great_circle_distance(const lla& a, const lla& b) {
    double dlat = b.lat_rad - a.lat_rad;
    double dlon = b.lon_rad - a.lon_rad;
    double h = std::sin(dlat/2) * std::sin(dlat/2)
             + std::cos(a.lat_rad) * std::cos(b.lat_rad) * std::sin(dlon/2) * std::sin(dlon/2);
    return 2.0 * constants::earth_radius_m * std::asin(std::sqrt(h));
}

// Flat-earth approximation: LLA difference to local NED offset (meters)
// Reference point is `ref`
inline vec3 lla_to_local_ned(const lla& ref, const lla& point) {
    double dlat = point.lat_rad - ref.lat_rad;
    double dlon = point.lon_rad - ref.lon_rad;
    double north = dlat * constants::earth_radius_m;
    double east  = dlon * constants::earth_radius_m * std::cos(ref.lat_rad);
    double down  = -(point.alt_m - ref.alt_m);
    return {north, east, down};
}

} // namespace xsf_math
