#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace xsf_math {

// Linear interpolation between two values
inline constexpr double lerp(double a, double b, double t) noexcept {
    return a + t * (b - a);
}

// Clamp value to [lo, hi]
inline constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 1D table lookup with linear interpolation
// xs must be sorted ascending. Clamps to endpoints.
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

// 2D table lookup: rows indexed by x, columns by y
// Returns bilinear interpolation.
inline double table_lookup_2d(const std::vector<double>& xs,
                              const std::vector<double>& ys,
                              const std::vector<std::vector<double>>& data,
                              double x, double y) {
    assert(xs.size() >= 2 && ys.size() >= 2);
    assert(data.size() == xs.size());

    // Find x interval
    double cx = clamp(x, xs.front(), xs.back());
    auto itx = std::lower_bound(xs.begin(), xs.end(), cx);
    size_t ix = static_cast<size_t>(itx - xs.begin());
    if (ix == 0) ix = 1;
    if (ix >= xs.size()) ix = xs.size() - 1;
    double tx = (cx - xs[ix-1]) / (xs[ix] - xs[ix-1]);

    // Find y interval
    double cy = clamp(y, ys.front(), ys.back());
    auto ity = std::lower_bound(ys.begin(), ys.end(), cy);
    size_t iy = static_cast<size_t>(ity - ys.begin());
    if (iy == 0) iy = 1;
    if (iy >= ys.size()) iy = ys.size() - 1;
    double ty = (cy - ys[iy-1]) / (ys[iy] - ys[iy-1]);

    // Bilinear
    double v00 = data[ix-1][iy-1];
    double v10 = data[ix][iy-1];
    double v01 = data[ix-1][iy];
    double v11 = data[ix][iy];

    return lerp(lerp(v00, v10, tx), lerp(v01, v11, tx), ty);
}

} // namespace xsf_math
