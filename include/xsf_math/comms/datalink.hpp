#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

// 通信链路端点参数（Link endpoint parameters）
struct link_endpoint {
    double tx_power_w = 50.0;                // 发射功率（Transmit power），单位 W
    double antenna_gain_db = 0.0;            // 天线增益（Antenna gain），单位 dB
    double receiver_noise_figure_db = 4.0;   // 接收机噪声系数（Receiver noise figure），单位 dB
    double bandwidth_hz = 1.0e6;             // 带宽（Bandwidth），单位 Hz
    double required_snr_db = 10.0;           // 所需信噪比（Required SNR），单位 dB
};

// 数据链接触信息（Datalink contact information）
struct datalink_contact {
    double free_space_loss_db = 0.0;     // 自由空间损耗（Free-space path loss），单位 dB
    double rx_power_dbw = 0.0;           // 接收功率（Received power），单位 dBW
    double noise_power_dbw = 0.0;        // 噪声功率（Noise power），单位 dBW
    double snr_db = 0.0;                 // 信噪比（Signal-to-Noise Ratio），单位 dB
    double link_margin_db = 0.0;         // 链路裕量（Link margin），单位 dB
    double one_way_latency_s = 0.0;      // 单向时延（One-way latency），单位 s
    double throughput_bps = 0.0;         // 吞吐量（Throughput），单位 bps
    bool link_up = false;                // 链路是否建立（Link up flag）
};

// 自由空间路径损耗（Free-space path loss, FSPL）
// range_m: 距离（m）; frequency_hz: 载波频率（Hz）
inline double free_space_path_loss_db(double range_m, double frequency_hz) {
    if (range_m <= 0.0 || frequency_hz <= 0.0) return 0.0;
    double wavelength = constants::speed_of_light / frequency_hz;  // 波长（wavelength），单位 m
    return 20.0 * std::log10(4.0 * constants::pi * range_m / wavelength);
}

// 数据链链路预算（Datalink link budget）
struct datalink_budget {
    double atmospheric_loss_db = 0.0;     // 大气损耗（Atmospheric loss），单位 dB
    double implementation_loss_db = 1.5;  // 实现损耗（Implementation loss），单位 dB
    double processing_gain_db = 0.0;      // 处理增益（Processing gain），单位 dB

    // 评估链路预算（Evaluate link budget）
    // tx: 发射端参数; rx: 接收端参数; range_m: 距离（m）; carrier_frequency_hz: 载波频率（Hz）
    // message_bits: 消息比特数（可选，用于计算附加时延）
    datalink_contact evaluate(const link_endpoint& tx,
                              const link_endpoint& rx,
                              double range_m,
                              double carrier_frequency_hz,
                              double message_bits = 0.0) const {
        datalink_contact out;
        out.free_space_loss_db = free_space_path_loss_db(range_m, carrier_frequency_hz);

        double tx_power_dbw = linear_to_db(std::max(tx.tx_power_w, 1.0e-20));
        out.rx_power_dbw = tx_power_dbw + tx.antenna_gain_db + rx.antenna_gain_db -
                           out.free_space_loss_db - atmospheric_loss_db - implementation_loss_db;

        double noise_power_w = constants::boltzmann_k * 290.0 * std::max(rx.bandwidth_hz, 1.0);  // 热噪声功率（W）
        out.noise_power_dbw = linear_to_db(noise_power_w) + rx.receiver_noise_figure_db;
        out.snr_db = out.rx_power_dbw - out.noise_power_dbw + processing_gain_db;
        out.link_margin_db = out.snr_db - rx.required_snr_db;
        out.link_up = out.link_margin_db >= 0.0;

        double snr_linear = db_to_linear(std::max(out.snr_db, -40.0));
        out.throughput_bps = rx.bandwidth_hz * std::log2(1.0 + snr_linear);  // 香农容量（Shannon capacity）
        out.one_way_latency_s = range_m / constants::speed_of_light;  // 光速传播时延
        if (message_bits > 0.0) {
            out.one_way_latency_s += message_bits / std::max(out.throughput_bps, 1.0);  // 传输时延
        }
        return out;
    }
};

// 时分多址请求（Time Division Multiple Access, TDMA request）
struct tdma_request {
    int node_id = -1;            // 节点标识（Node ID）
    double message_bits = 0.0;   // 消息比特数（Message bits）
    double priority = 1.0;       // 优先级（Priority），数值越大优先级越高
};

// 时分多址时隙（TDMA slot）
struct tdma_slot {
    int node_id = -1;            // 分配的节点标识（Assigned node ID）
    double start_time_s = 0.0;   // 时隙起始时间（Slot start time），单位 s
    double duration_s = 0.0;     // 时隙持续时间（Slot duration），单位 s
    double allocated_bits = 0.0; // 分配的比特数（Allocated bits）
};

// 时分多址调度器（TDMA scheduler）
struct tdma_scheduler {
    double frame_duration_s = 1.0;   // 帧时长（Frame duration），单位 s
    double guard_interval_s = 1.0e-3;// 保护间隔（Guard interval），单位 s
    int max_slots = 32;              // 每帧最大时隙数（Maximum slots per frame）

    // 分配时隙（Allocate TDMA slots）
    // requests: 请求列表; usable_rate_bps: 可用数据速率（bps）
    std::vector<tdma_slot> allocate(std::vector<tdma_request> requests,
                                    double usable_rate_bps) const {
        std::vector<tdma_slot> out;
        if (usable_rate_bps <= 0.0 || frame_duration_s <= 0.0) return out;

        std::sort(requests.begin(), requests.end(), [](const tdma_request& a, const tdma_request& b) {
            if (a.priority == b.priority) return a.node_id < b.node_id;
            return a.priority > b.priority;
        });

        double cursor = 0.0;
        for (const auto& req : requests) {
            if (static_cast<int>(out.size()) >= max_slots) break;
            double duration = req.message_bits / usable_rate_bps;
            if (duration <= 0.0) continue;
            if (cursor + duration > frame_duration_s) break;
            out.push_back({req.node_id, cursor, duration, req.message_bits});
            cursor += duration + guard_interval_s;
        }
        return out;
    }
};

}  // namespace xsf_math
