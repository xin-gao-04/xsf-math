#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace xsf_math {

/** 两个值之间的线性插值 (Linear interpolation between two values) */
inline constexpr double lerp(double a, double b, double t) noexcept {
    return a + t * (b - a);
}

/** 将数值限制到 [lo, hi] 区间 (Clamp value to [lo, hi]) */
inline constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

/** 一维表查找并进行线性插值 (1D table lookup with linear interpolation) */
// xs 必须升序排列。超出范围时钳制到端点。
inline double table_lookup(const std::vector<double>& xs,
                           const std::vector<double>& ys,
                           double x) {
    assert(xs.size() == ys.size() && xs.size() >= 2);

    if (x <= xs.front()) return ys.front();
    if (x >= xs.back())  return ys.back();

    auto it = std::lower_bound(xs.begin(), xs.end(), x);
    size_t i = static_cast<size_t>(it - xs.begin());
    if (i == 0) i = 1;

    double t = (x - xs[i-1]) / (xs[i] - xs[i-1]);
    return lerp(ys[i-1], ys[i], t);
}

/** 二维表查找：行按 x 索引，列按 y 索引，返回双线性插值结果 (2D table lookup: rows indexed by x, columns by y, returns bilinear interpolation result) */
inline double table_lookup_2d(const std::vector<double>& xs,
                              const std::vector<double>& ys,
                              const std::vector<std::vector<double>>& data,
                              double x, double y) {
    assert(xs.size() >= 2 && ys.size() >= 2);
    assert(data.size() == xs.size());

    // 找到 x 区间 (Find x interval)
    double cx = clamp(x, xs.front(), xs.back());
    auto itx = std::lower_bound(xs.begin(), xs.end(), cx);
    size_t ix = static_cast<size_t>(itx - xs.begin());
    if (ix == 0) ix = 1;
    if (ix >= xs.size()) ix = xs.size() - 1;
    double tx = (cx - xs[ix-1]) / (xs[ix] - xs[ix-1]);

    // 找到 y 区间 (Find y interval)
    double cy = clamp(y, ys.front(), ys.back());
    auto ity = std::lower_bound(ys.begin(), ys.end(), cy);
    size_t iy = static_cast<size_t>(ity - ys.begin());
    if (iy == 0) iy = 1;
    if (iy >= ys.size()) iy = ys.size() - 1;
    double ty = (cy - ys[iy-1]) / (ys[iy] - ys[iy-1]);

    // 双线性插值 (Bilinear interpolation)
    double v00 = data[ix-1][iy-1];
    double v10 = data[ix][iy-1];
    double v01 = data[ix-1][iy];
    double v11 = data[ix][iy];

    return lerp(lerp(v00, v10, tx), lerp(v01, v11, tx), ty);
}

} // namespace xsf_math
