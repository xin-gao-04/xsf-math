#pragma once

#include <cmath>
#include <array>
#include <stdexcept>

namespace xsf_math {

/** 三维向量 (3D vector) */
struct vec3 {
    double x = 0.0; ///< X 分量 (X component)
    double y = 0.0; ///< Y 分量 (Y component)
    double z = 0.0; ///< Z 分量 (Z component)

    /** 默认构造函数 (Default constructor) */
    constexpr vec3() = default;

    /** 按分量构造 (Construct from components) */
    constexpr vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}

    /** 从 double[3] 数组构造 (Construct from double[3] array) */
    explicit constexpr vec3(const double v[3]) : x(v[0]), y(v[1]), z(v[2]) {}

    /** 下标访问，返回引用 (Subscript access, returns reference) */
    constexpr double& operator[](int i) {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }

    /** 下标访问，返回常量值 (Subscript access, returns value) */
    constexpr double operator[](int i) const {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }

    /** 向量加法 (Vector addition) */
    constexpr vec3 operator+(const vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }

    /** 向量减法 (Vector subtraction) */
    constexpr vec3 operator-(const vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }

    /** 数乘 (Scalar multiplication) */
    constexpr vec3 operator*(double s) const { return {x*s, y*s, z*s}; }

    /** 数除 (Scalar division) */
    constexpr vec3 operator/(double s) const { return {x/s, y/s, z/s}; }

    /** 取反 (Negation) */
    constexpr vec3 operator-() const { return {-x, -y, -z}; }

    /** 累加赋值 (Addition assignment) */
    constexpr vec3& operator+=(const vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }

    /** 累减赋值 (Subtraction assignment) */
    constexpr vec3& operator-=(const vec3& b) { x-=b.x; y-=b.y; z-=b.z; return *this; }

    /** 数乘赋值 (Scalar multiplication assignment) */
    constexpr vec3& operator*=(double s) { x*=s; y*=s; z*=s; return *this; }

    /** 点积 (Dot product) */
    constexpr double dot(const vec3& b) const { return x*b.x + y*b.y + z*b.z; }

    /** 叉积 (Cross product) */
    constexpr vec3 cross(const vec3& b) const {
        return { y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x };
    }

    /** 模长 (Magnitude) */
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }

    /** 模长平方 (Magnitude squared) */
    double magnitude_sq() const { return x*x + y*y + z*z; }

    /** 归一化：返回同方向单位向量；若模长过小则返回零向量 (Normalized: returns unit vector; returns zero if magnitude is too small) */
    vec3 normalized() const {
        double m = magnitude();
        if (m < 1e-30) return {0, 0, 0};
        return *this / m;
    }

    /** 输出到 double[3] 数组 (Write to double[3] array) */
    void to_array(double out[3]) const { out[0] = x; out[1] = y; out[2] = z; }
};

/** 左乘标量 (Left scalar multiplication) */
inline constexpr vec3 operator*(double s, const vec3& v) { return v * s; }

} // namespace xsf_math
