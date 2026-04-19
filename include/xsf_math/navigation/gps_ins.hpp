#pragma once

#include "../core/constants.hpp"
#include "../core/coordinate_transform.hpp"
#include "../core/quaternion.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

// 惯性测量单元采样（Inertial Measurement Unit, IMU sample）
struct imu_sample {
    vec3 specific_force_body_mps2{};  // 比力（Specific force）在弹体坐标系下的分量，单位 m/s^2
    vec3 gyro_body_radps{};           // 角速度（Angular rate）在弹体坐标系下的分量，单位 rad/s
    double dt_s = 0.01;               // 采样时间间隔（Sampling interval），单位 s
};

// GPS 测量值（GPS measurement）
struct gps_measurement {
    bool valid = false;               // 测量是否有效（Measurement valid flag）
    vec3 position_wcs{};              // WCS（世界坐标系）位置（Position in world coordinate system），单位 m
    vec3 velocity_wcs{};              // WCS 速度（Velocity in world coordinate system），单位 m/s
    double position_sigma_m = 5.0;    // 位置测量标准差（Position measurement sigma），单位 m
    double velocity_sigma_mps = 0.5;  // 速度测量标准差（Velocity measurement sigma），单位 m/s
};

// GPS/INS 组合导航状态（GPS/INS integrated navigation state）
struct gps_ins_state {
    vec3 position_wcs{};                         // WCS 位置（Position），单位 m
    vec3 velocity_wcs{};                         // WCS 速度（Velocity），单位 m/s
    quaternion attitude_body_to_wcs{};           // 弹体到 WCS 的姿态四元数（Attitude quaternion body->WCS）
    vec3 accel_bias_body_mps2{};                 // 加速度计零偏（Accelerometer bias），弹体坐标系，单位 m/s^2
    vec3 gyro_bias_body_radps{};                 // 陀螺零偏（Gyro bias），弹体坐标系，单位 rad/s
};

// GPS 可见性评估结果（GPS visibility assessment result）
struct gps_visibility {
    int satellites_visible = 0;   // 可见卫星数（Visible satellites count）
    double gdop = 99.0;           // 几何精度因子（Geometric Dilution of Precision, GDOP）
    bool usable = false;          // 是否可用（Usable flag）
};

// GPS/INS 松耦合滤波器（GPS/INS loose coupler）
struct gps_ins_loose_coupler {
    double position_gain = 0.2;   // 位置修正增益（Position correction gain）
    double velocity_gain = 0.3;   // 速度修正增益（Velocity correction gain）
    double bias_gain = 0.01;      // 零偏修正增益（Bias correction gain）

    // 利用 IMU 数据 propagate 状态（Propagate state using IMU data）
    void propagate(gps_ins_state& state, const imu_sample& imu) const {
        if (imu.dt_s <= 0.0) return;

        vec3 gyro = imu.gyro_body_radps - state.gyro_bias_body_radps;  // 补偿零偏后的角速度（Bias-corrected angular rate）
        quaternion omega_q{0.0, gyro.x, gyro.y, gyro.z};
        quaternion qdot = (state.attitude_body_to_wcs * omega_q) * 0.5;  // 四元数导数（Quaternion derivative）
        state.attitude_body_to_wcs = (state.attitude_body_to_wcs + qdot * imu.dt_s).normalized();

        vec3 corrected_force = imu.specific_force_body_mps2 - state.accel_bias_body_mps2;  // 补偿零偏后的比力
        vec3 accel_wcs = state.attitude_body_to_wcs.rotate(corrected_force) + vec3{0.0, 0.0, constants::gravity_mps2};
        state.velocity_wcs += accel_wcs * imu.dt_s;
        state.position_wcs += state.velocity_wcs * imu.dt_s;
    }

    // 利用 GPS 测量值更新状态（Update state using GPS measurement）
    void update(gps_ins_state& state, const gps_measurement& gps) const {
        if (!gps.valid) return;
        vec3 pos_residual = gps.position_wcs - state.position_wcs;  // 位置残差（Position residual）
        vec3 vel_residual = gps.velocity_wcs - state.velocity_wcs;  // 速度残差（Velocity residual）
        state.position_wcs += pos_residual * position_gain;
        state.velocity_wcs += vel_residual * velocity_gain;
        state.accel_bias_body_mps2 +=
            state.attitude_body_to_wcs.inverse().rotate(vel_residual) * (bias_gain / std::max(gps.velocity_sigma_mps, 1.0));
    }
};

// 估计 GPS 可见性（Estimate GPS visibility）
// satellite_positions_ecef: 卫星 ECEF 位置列表（m）; receiver_position_ecef: 接收机 ECEF 位置（m）
// min_elevation_rad: 最小仰角（rad）
inline gps_visibility estimate_gps_visibility(const std::vector<vec3>& satellite_positions_ecef,
                                              const vec3& receiver_position_ecef,
                                              double min_elevation_rad = 10.0 * constants::deg_to_rad) {
    gps_visibility out;
    vec3 up = receiver_position_ecef.normalized();  // 当地天向单位矢量（Local up direction）
    for (const auto& sat : satellite_positions_ecef) {
        vec3 los = (sat - receiver_position_ecef).normalized();  // 视线方向单位矢量（Line-of-sight unit vector）
        double elevation = std::asin(std::clamp(los.dot(up), -1.0, 1.0));  // 仰角（Elevation angle），单位 rad
        if (elevation >= min_elevation_rad) ++out.satellites_visible;
    }
    out.usable = out.satellites_visible >= 4;  // 至少需要 4 颗卫星定位
    out.gdop = out.usable ? 4.0 / std::sqrt(static_cast<double>(out.satellites_visible)) : 99.0;
    return out;
}

}  // namespace xsf_math
