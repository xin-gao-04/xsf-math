#pragma once

#include "../core/constants.hpp"
#include "../core/rcs.hpp"
#include <algorithm>
#include <cmath>

namespace xsf_math {

/** 球体 RCS 模型：瑞利区与光学区近似 (Sphere RCS model: Rayleigh and optical region approximations) */
struct sphere_rcs {
    double radius_m = 1.0; ///< 球体半径 (sphere radius)，单位 m

    /** 评估球体 RCS (Evaluate sphere RCS) */
    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double size_ratio = radius_m / lambda;
        if (size_ratio < 0.2) {
            // 瑞利区近似：sigma 正比于 r^6/lambda^4 (Rayleigh region approximation: sigma proportional to r^6/lambda^4)
            double sigma = 9.0 * constants::pi * std::pow(radius_m, 6) / std::pow(lambda, 4);
            return rcs_value::from_m2(std::max(sigma, 0.0));
        }
        // 光学区：几何截面 pi*r^2 (Optical region: geometric cross-section pi*r^2)
        return rcs_value::from_m2(constants::pi * radius_m * radius_m);
    }
};

/** 平板 RCS 模型（物理光学近似） (Flat plate RCS model, physical optics approximation) */
struct flat_plate_rcs {
    double width_m = 1.0;  ///< 平板宽度 (plate width)，单位 m
    double height_m = 1.0; ///< 平板高度 (plate height)，单位 m

    /** 评估平板 RCS，入射角由方位角和俯仰角共同决定 (Evaluate flat plate RCS, incidence angle from azimuth and elevation) */
    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        double area = width_m * height_m;
        // 入射余弦：法向投影因子 (Incidence cosine: normal projection factor)
        double cos_inc = std::max(std::cos(az_rad) * std::cos(el_rad), 0.0);
        // 物理光学公式：sigma = 4*pi*A^2*cos^2(theta_inc)/lambda^2 (Physical optics formula)
        double sigma = 4.0 * constants::pi * area * area * cos_inc * cos_inc / (lambda * lambda);
        return rcs_value::from_m2(std::max(sigma, 0.0));
    }
};

/** 圆柱体 RCS 模型（侧面照射近似） (Cylinder RCS model, broadside illumination approximation) */
struct cylinder_rcs {
    double radius_m = 0.5; ///< 圆柱半径 (cylinder radius)，单位 m
    double length_m = 4.0; ///< 圆柱长度 (cylinder length)，单位 m

    /** 评估圆柱体 RCS，aspect 因子决定侧面回波强度 (Evaluate cylinder RCS, aspect factor determines broadside echo strength) */
    rcs_value evaluate(double az_rad, double el_rad, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        // 侧面照射基础 RCS：2*pi*r*L^2/lambda (Broadside base RCS)
        double broadside = 2.0 * constants::pi * radius_m * length_m * length_m / std::max(lambda, 1.0e-9);
        // 姿态因子：sin^2(az)*cos^2(el) (Aspect factor)
        double aspect = std::sin(az_rad) * std::sin(az_rad) * std::cos(el_rad) * std::cos(el_rad);
        return rcs_value::from_m2(std::max(broadside * aspect, 0.0));
    }
};

/** 角反射器 RCS 模型（三面直角反射器） (Corner reflector RCS model, trihedral corner reflector) */
struct corner_reflector_rcs {
    double leg_length_m = 1.0; ///< 直角边长度 (leg length)，单位 m

    /** 评估角反射器 RCS（与视角无关，近似） (Evaluate corner reflector RCS, approximately aspect-independent) */
    rcs_value evaluate(double /*az_rad*/, double /*el_rad*/, double freq_hz) const {
        double lambda = constants::speed_of_light / std::max(freq_hz, 1.0);
        // 三面角反射器公式：sigma = 12*pi*L^4/lambda^2 (Trihedral corner reflector formula)
        double sigma = 12.0 * constants::pi * std::pow(leg_length_m, 4) / std::max(lambda * lambda, 1.0e-12);
        return rcs_value::from_m2(std::max(sigma, 0.0));
    }
};

}  // namespace xsf_math
