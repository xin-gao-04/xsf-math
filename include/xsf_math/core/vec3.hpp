#pragma once

#include <cmath>
#include <array>
#include <stdexcept>

namespace xsf_math {

struct vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    constexpr vec3() = default;
    constexpr vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
    explicit constexpr vec3(const double v[3]) : x(v[0]), y(v[1]), z(v[2]) {}

    constexpr double& operator[](int i) {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }
    constexpr double operator[](int i) const {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }

    constexpr vec3 operator+(const vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    constexpr vec3 operator-(const vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    constexpr vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    constexpr vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    constexpr vec3 operator-() const { return {-x, -y, -z}; }

    constexpr vec3& operator+=(const vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    constexpr vec3& operator-=(const vec3& b) { x-=b.x; y-=b.y; z-=b.z; return *this; }
    constexpr vec3& operator*=(double s) { x*=s; y*=s; z*=s; return *this; }

    constexpr double dot(const vec3& b) const { return x*b.x + y*b.y + z*b.z; }

    constexpr vec3 cross(const vec3& b) const {
        return { y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x };
    }

    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
    double magnitude_sq() const { return x*x + y*y + z*z; }

    vec3 normalized() const {
        double m = magnitude();
        if (m < 1e-30) return {0, 0, 0};
        return *this / m;
    }

    void to_array(double out[3]) const { out[0] = x; out[1] = y; out[2] = z; }
};

inline constexpr vec3 operator*(double s, const vec3& v) { return v * s; }

} // namespace xsf_math
