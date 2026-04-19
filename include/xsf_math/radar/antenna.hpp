#pragma once

#include "../core/constants.hpp"
#include "../core/vec3.hpp"
#include "../core/mat3.hpp"
#include "../core/interpolation.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace xsf_math {

// 天线增益方向图模型 (Antenna Gain Pattern Models)

// 天线增益评估结果 (Antenna gain evaluation result)
struct antenna_gain_result {
    double gain_linear = 1.0;  // 线性功率增益 (Linear power gain)
    double gain_db     = 0.0;  // dBi 增益 (Gain in dBi)
    bool   in_fov      = true; // 是否在视场内 (Whether inside field of view)

    // 由 dBi 值构造结果 (Construct result from dBi value)
    static antenna_gain_result from_db(double db, bool fov = true) {
        return { db_to_linear(db), db, fov };
    }
    // 由线性值构造结果 (Construct result from linear value)
    static antenna_gain_result from_linear(double lin, bool fov = true) {
        return { lin, linear_to_db(lin), fov };
    }
};

// 各向同性天线（各方向均为 0 dBi）(Isotropic antenna: 0 dBi in all directions)
struct antenna_isotropic {
    // 评估给定方位/俯仰偏移角处的增益 (Evaluate gain at given azimuth/elevation offset angles)
    antenna_gain_result evaluate(double /*az_off_rad*/, double /*el_off_rad*/) const {
        return {1.0, 0.0, true};
    }
};

// 余弦幂方向图（简单解析模型）(Cosine-power pattern: simple analytical model)
// G(theta) = G_peak * cos^n(theta)，当 |theta| < half_beamwidth
struct antenna_cosine {
    double peak_gain_db   = 30.0;   // 峰值增益（dBi）(Peak gain in dBi)
    double half_bw_az_rad = 5.0 * constants::deg_to_rad;  // 方位半功率波束宽度（rad）(Azimuth half-power beamwidth)
    double half_bw_el_rad = 5.0 * constants::deg_to_rad;  // 俯仰半功率波束宽度（rad）(Elevation half-power beamwidth)
    double fov_az_rad     = 60.0 * constants::deg_to_rad; // 方位视场半角（rad）(Azimuth field-of-view half-angle)
    double fov_el_rad     = 60.0 * constants::deg_to_rad; // 俯仰视场半角（rad）(Elevation field-of-view half-angle)
    double sidelobe_db    = -20.0;  // 平均旁瓣电平（dBi）(Average sidelobe level)

    // 评估给定偏移角处的增益 (Evaluate gain at specified offset angles)
    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);

        if (!in_fov) {
            return antenna_gain_result::from_db(sidelobe_db - 10.0, false);
        }

        // 从波束轴向开始的余弦衰减 (Cosine attenuation from beam axis)
        // 在半波束宽度处，增益下降 3 dB (Gain drops 3 dB at half beamwidth)
        // n = ln(0.5) / ln(cos(half_bw)) (Exponent n derived from half-power condition)
        double az_norm = (half_bw_az_rad > 1e-10) ? az_off_rad / half_bw_az_rad : 0.0;
        double el_norm = (half_bw_el_rad > 1e-10) ? el_off_rad / half_bw_el_rad : 0.0;

        double theta_norm = std::sqrt(az_norm*az_norm + el_norm*el_norm);

        double gain_db;
        if (theta_norm < 0.01) {
            gain_db = peak_gain_db;
        } else if (theta_norm <= 1.0) {
            // 主瓣：类似高斯的衰减 (Main lobe: Gaussian-like attenuation)
            gain_db = peak_gain_db - 3.0 * theta_norm * theta_norm;
        } else {
            // 超出主瓣：过渡到旁瓣 (Beyond main lobe: transition to sidelobe)
            double main_beam_edge = peak_gain_db - 3.0;
            double sll = sidelobe_db;
            double t = std::min((theta_norm - 1.0) / 2.0, 1.0);
            gain_db = lerp(main_beam_edge, sll, t);
        }

        return antenna_gain_result::from_db(gain_db, in_fov);
    }
};

// sinc 平方方向图（均匀矩形孔径的典型形式）(Sinc-squared pattern: typical for uniform rectangular aperture)
// G(theta) = G_peak * sinc^2(pi * D * sin(theta) / lambda)
struct antenna_sinc {
    double peak_gain_db   = 30.0;  // 峰值增益（dBi）(Peak gain in dBi)
    double aperture_az_m  = 1.0;   // 方位向孔径尺寸（m）(Aperture dimension in azimuth)
    double aperture_el_m  = 1.0;   // 俯仰向孔径尺寸（m）(Aperture dimension in elevation)
    double frequency_hz   = 10.0e9; // 工作频率（Hz）(Operating frequency)
    double fov_az_rad     = 60.0 * constants::deg_to_rad; // 方位视场半角（rad）(Azimuth FOV half-angle)
    double fov_el_rad     = 60.0 * constants::deg_to_rad; // 俯仰视场半角（rad）(Elevation FOV half-angle)

    // 评估给定偏移角处的增益 (Evaluate gain at specified offset angles)
    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);
        if (!in_fov) {
            return antenna_gain_result::from_db(-40.0, false);
        }

        double lambda = constants::speed_of_light / frequency_hz;

        auto sinc = [](double x) -> double {
            if (std::abs(x) < 1e-10) return 1.0;
            double px = constants::pi * x;
            return std::sin(px) / px;
        };

        // 空间频率 u = D * sin(theta) / lambda (Spatial frequency)
        double u_az = aperture_az_m * std::sin(az_off_rad) / lambda;
        double u_el = aperture_el_m * std::sin(el_off_rad) / lambda;

        double pattern = sinc(u_az) * sinc(u_el);
        double gain_linear = db_to_linear(peak_gain_db) * pattern * pattern;

        if (gain_linear < 1e-10) gain_linear = 1e-10;
        return antenna_gain_result::from_linear(gain_linear, in_fov);
    }
};

// 查表天线方向图（来自实测或仿真数据）(Table-lookup antenna pattern from measured or simulated data)
struct antenna_table {
    double peak_gain_db = 30.0; // 峰值增益（dBi）(Peak gain in dBi)
    std::vector<double> az_angles_rad;    // 方位角采样点（已排序）(Azimuth angle samples, sorted)
    std::vector<double> el_angles_rad;    // 俯仰角采样点（已排序）(Elevation angle samples, sorted)
    std::vector<std::vector<double>> pattern_db;  // [az][el]，相对于峰值（dB）(Pattern relative to peak)

    double fov_az_rad = 60.0 * constants::deg_to_rad; // 方位视场半角（rad）(Azimuth FOV half-angle)
    double fov_el_rad = 60.0 * constants::deg_to_rad; // 俯仰视场半角（rad）(Elevation FOV half-angle)

    // 评估给定偏移角处的增益（二维插值）(Evaluate gain at offset angles via 2D interpolation)
    antenna_gain_result evaluate(double az_off_rad, double el_off_rad) const {
        bool in_fov = (std::abs(az_off_rad) <= fov_az_rad) &&
                      (std::abs(el_off_rad) <= fov_el_rad);

        double relative_db = table_lookup_2d(az_angles_rad, el_angles_rad,
                                              pattern_db, az_off_rad, el_off_rad);
        double gain_db = peak_gain_db + relative_db;

        return antenna_gain_result::from_db(gain_db, in_fov);
    }
};

// 由孔径面积计算峰值增益 (Calculate peak gain from aperture area)
inline double aperture_gain_db(double aperture_area_m2, double frequency_hz, double efficiency = 0.55) {
    double lambda = constants::speed_of_light / frequency_hz;
    // 增益公式：G = efficiency * 4*pi*A / lambda^2 (Gain formula)
    double gain = efficiency * 4.0 * constants::pi * aperture_area_m2 / (lambda * lambda);
    return linear_to_db(gain);
}

// 根据孔径尺寸计算半功率波束宽度（rad）(Calculate half-power beamwidth from aperture dimension)
inline double beamwidth_rad(double aperture_dim_m, double frequency_hz) {
    double lambda = constants::speed_of_light / frequency_hz;
    return 0.886 * lambda / aperture_dim_m;  // ~51 deg * lambda/D
}

} // namespace xsf_math
