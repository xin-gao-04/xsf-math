#pragma once

#include "../core/constants.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace xsf_math {

struct link_endpoint {
    double tx_power_w = 50.0;
    double antenna_gain_db = 0.0;
    double receiver_noise_figure_db = 4.0;
    double bandwidth_hz = 1.0e6;
    double required_snr_db = 10.0;
};

struct datalink_contact {
    double free_space_loss_db = 0.0;
    double rx_power_dbw = 0.0;
    double noise_power_dbw = 0.0;
    double snr_db = 0.0;
    double link_margin_db = 0.0;
    double one_way_latency_s = 0.0;
    double throughput_bps = 0.0;
    bool link_up = false;
};

inline double free_space_path_loss_db(double range_m, double frequency_hz) {
    if (range_m <= 0.0 || frequency_hz <= 0.0) return 0.0;
    double wavelength = constants::speed_of_light / frequency_hz;
    return 20.0 * std::log10(4.0 * constants::pi * range_m / wavelength);
}

struct datalink_budget {
    double atmospheric_loss_db = 0.0;
    double implementation_loss_db = 1.5;
    double processing_gain_db = 0.0;

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

        double noise_power_w = constants::boltzmann_k * 290.0 * std::max(rx.bandwidth_hz, 1.0);
        out.noise_power_dbw = linear_to_db(noise_power_w) + rx.receiver_noise_figure_db;
        out.snr_db = out.rx_power_dbw - out.noise_power_dbw + processing_gain_db;
        out.link_margin_db = out.snr_db - rx.required_snr_db;
        out.link_up = out.link_margin_db >= 0.0;

        double snr_linear = db_to_linear(std::max(out.snr_db, -40.0));
        out.throughput_bps = rx.bandwidth_hz * std::log2(1.0 + snr_linear);
        out.one_way_latency_s = range_m / constants::speed_of_light;
        if (message_bits > 0.0) {
            out.one_way_latency_s += message_bits / std::max(out.throughput_bps, 1.0);
        }
        return out;
    }
};

struct tdma_request {
    int node_id = -1;
    double message_bits = 0.0;
    double priority = 1.0;
};

struct tdma_slot {
    int node_id = -1;
    double start_time_s = 0.0;
    double duration_s = 0.0;
    double allocated_bits = 0.0;
};

struct tdma_scheduler {
    double frame_duration_s = 1.0;
    double guard_interval_s = 1.0e-3;
    int max_slots = 32;

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
