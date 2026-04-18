#pragma once

#include "constants.hpp"
#include "mat3.hpp"
#include "vec3.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

struct euler_angles;

// 轻量四元数：q = w + xi + yj + zk
struct quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double magnitude() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    quaternion normalized() const {
        double mag = magnitude();
        if (mag < 1e-20) return {};
        return {w / mag, x / mag, y / mag, z / mag};
    }

    quaternion conjugate() const { return {w, -x, -y, -z}; }

    quaternion inverse() const {
        double n2 = w * w + x * x + y * y + z * z;
        if (n2 < 1e-20) return {};
        quaternion c = conjugate();
        return {c.w / n2, c.x / n2, c.y / n2, c.z / n2};
    }

    quaternion operator*(const quaternion& b) const {
        return {
            w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w
        };
    }

    quaternion operator*(double s) const { return {w * s, x * s, y * s, z * s}; }
    quaternion operator+(const quaternion& b) const { return {w + b.w, x + b.x, y + b.y, z + b.z}; }
    quaternion operator-(const quaternion& b) const { return {w - b.w, x - b.x, y - b.y, z - b.z}; }

    vec3 rotate(const vec3& v) const {
        quaternion qv{0.0, v.x, v.y, v.z};
        quaternion qr = (*this).normalized() * qv * this->normalized().conjugate();
        return {qr.x, qr.y, qr.z};
    }

    static quaternion from_axis_angle(const vec3& axis, double angle_rad) {
        vec3 unit = axis.normalized();
        double half = 0.5 * angle_rad;
        double s = std::sin(half);
        return {std::cos(half), unit.x * s, unit.y * s, unit.z * s};
    }

    static quaternion from_dcm(const mat3& dcm) {
        double trace = dcm.m[0][0] + dcm.m[1][1] + dcm.m[2][2];
        quaternion q;
        if (trace > 0.0) {
            double s = std::sqrt(trace + 1.0) * 2.0;
            q.w = 0.25 * s;
            q.x = (dcm.m[2][1] - dcm.m[1][2]) / s;
            q.y = (dcm.m[0][2] - dcm.m[2][0]) / s;
            q.z = (dcm.m[1][0] - dcm.m[0][1]) / s;
        } else if (dcm.m[0][0] > dcm.m[1][1] && dcm.m[0][0] > dcm.m[2][2]) {
            double s = std::sqrt(1.0 + dcm.m[0][0] - dcm.m[1][1] - dcm.m[2][2]) * 2.0;
            q.w = (dcm.m[2][1] - dcm.m[1][2]) / s;
            q.x = 0.25 * s;
            q.y = (dcm.m[0][1] + dcm.m[1][0]) / s;
            q.z = (dcm.m[0][2] + dcm.m[2][0]) / s;
        } else if (dcm.m[1][1] > dcm.m[2][2]) {
            double s = std::sqrt(1.0 + dcm.m[1][1] - dcm.m[0][0] - dcm.m[2][2]) * 2.0;
            q.w = (dcm.m[0][2] - dcm.m[2][0]) / s;
            q.x = (dcm.m[0][1] + dcm.m[1][0]) / s;
            q.y = 0.25 * s;
            q.z = (dcm.m[1][2] + dcm.m[2][1]) / s;
        } else {
            double s = std::sqrt(1.0 + dcm.m[2][2] - dcm.m[0][0] - dcm.m[1][1]) * 2.0;
            q.w = (dcm.m[1][0] - dcm.m[0][1]) / s;
            q.x = (dcm.m[0][2] + dcm.m[2][0]) / s;
            q.y = (dcm.m[1][2] + dcm.m[2][1]) / s;
            q.z = 0.25 * s;
        }
        return q.normalized();
    }

    mat3 to_dcm() const {
        quaternion q = normalized();
        double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        mat3 r;
        r.m[0][0] = 1.0 - 2.0 * (yy + zz);
        r.m[0][1] = 2.0 * (xy - wz);
        r.m[0][2] = 2.0 * (xz + wy);
        r.m[1][0] = 2.0 * (xy + wz);
        r.m[1][1] = 1.0 - 2.0 * (xx + zz);
        r.m[1][2] = 2.0 * (yz - wx);
        r.m[2][0] = 2.0 * (xz - wy);
        r.m[2][1] = 2.0 * (yz + wx);
        r.m[2][2] = 1.0 - 2.0 * (xx + yy);
        return r;
    }
};

inline quaternion slerp(const quaternion& a, const quaternion& b, double t) {
    quaternion qa = a.normalized();
    quaternion qb = b.normalized();
    double dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;

    if (dot < 0.0) {
        qb = qb * -1.0;
        dot = -dot;
    }

    if (dot > 0.9995) return (qa + (qb - qa) * t).normalized();

    double theta0 = std::acos(std::clamp(dot, -1.0, 1.0));
    double theta = theta0 * t;
    quaternion qc = (qb - qa * dot).normalized();
    return qa * std::cos(theta) + qc * std::sin(theta);
}

}  // namespace xsf_math
