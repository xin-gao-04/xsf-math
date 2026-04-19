#pragma once

#include "vec3.hpp"
#include <cmath>

namespace xsf_math {

/** 3x3 矩阵，按行主序存储 (3x3 matrix stored row-major) */
// 3x3 matrix stored row-major: m[row][col]
struct mat3 {
    double m[3][3] = {}; ///< 矩阵元素，m[row][col] (matrix elements)

    /** 默认构造函数 (Default constructor) */
    constexpr mat3() = default;

    /** 返回单位矩阵 (Returns identity matrix) */
    static constexpr mat3 identity() {
        mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }

    /** 返回零矩阵 (Returns zero matrix) */
    static constexpr mat3 zero() { return mat3{}; }

    // Construct from rows
    /** 由三个行向量构造 (Construct from three row vectors) */
    static constexpr mat3 from_rows(const vec3& r0, const vec3& r1, const vec3& r2) {
        mat3 r;
        r.m[0][0]=r0.x; r.m[0][1]=r0.y; r.m[0][2]=r0.z;
        r.m[1][0]=r1.x; r.m[1][1]=r1.y; r.m[1][2]=r1.z;
        r.m[2][0]=r2.x; r.m[2][1]=r2.y; r.m[2][2]=r2.z;
        return r;
    }

    /** 矩阵与向量相乘 (Matrix-vector multiplication) */
    constexpr vec3 operator*(const vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    /** 矩阵与矩阵相乘 (Matrix-matrix multiplication) */
    constexpr mat3 operator*(const mat3& b) const {
        mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[i][0]*b.m[0][j] + m[i][1]*b.m[1][j] + m[i][2]*b.m[2][j];
        return r;
    }

    /** 转置矩阵 (Transposed matrix) */
    constexpr mat3 transposed() const {
        mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    /** 行列式 (Determinant) */
    constexpr double determinant() const {
        return m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
             - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
             + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    }
};

} // namespace xsf_math
