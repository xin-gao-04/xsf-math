#pragma once

#include "../core/interpolation.hpp"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace xsf_math {

// 发射杀伤概率查询请求（Launch Pk lookup request）
struct launch_pk_request {
    std::string launcher_type;     // 发射平台类型（Launcher type）
    std::string target_type;       // 目标类型（Target type）
    double altitude_m         = 0.0;   // 目标高度（Altitude），单位 m
    double target_speed_mps   = 0.0;   // 目标速度（Target speed），单位 m/s
    double down_range_m       = 0.0;   // 纵距（Down-range distance），单位 m
    double cross_range_m      = 0.0;   // 横距（Cross-range distance），单位 m
};

// 纵距-横距（Cross-Range / Down-Range, CR/DR）杀伤概率表 (CR/DR Pk table)
struct launch_pk_crdr_table {
    std::vector<double> cross_ranges_m;                       // 横距样本点（Cross-range samples），单位 m
    std::vector<double> down_ranges_m;                        // 纵距样本点（Down-range samples），单位 m
    std::vector<std::vector<double>> pk_grid; // 杀伤概率网格（Pk grid）[down_range][cross_range]

    // 二维查表评估 Pk（2D table lookup for Pk）
    double evaluate(double down_range_m, double cross_range_m) const {
        return table_lookup_2d(down_ranges_m, cross_ranges_m, pk_grid, down_range_m, cross_range_m);
    }
};

// 发射杀伤概率条目（Launch Pk entry for a given altitude and speed）
struct launch_pk_entry {
    double altitude_m       = 0.0;   // 高度（Altitude），单位 m
    double target_speed_mps = 0.0;   // 目标速度（Target speed），单位 m/s
    launch_pk_crdr_table table;      // 对应的 CR/DR 杀伤概率表（CR/DR Pk table）
};

// 发射杀伤概率表集合（Launch Pk table set）
class launch_pk_table_set {
public:
    double default_pk = 0.0;  // 默认杀伤概率（Default Pk）

    // 从核心数据文件加载单张表（Load a single table from core data file）
    bool load_core_file(const std::string& file_path, std::string* error = nullptr) {
        std::ifstream input(file_path);
        if (!input) {
            if (error) *error = "failed to open file";
            return false;
        }

        std::string launcher_type;
        std::string target_type;
        std::string length_units;
        std::string speed_units;
        double altitude_value = 0.0;
        double speed_value    = 0.0;

        if (!read_header(input, "Site Platform Type", launcher_type, error) ||
            !read_header(input, "Target Platform Type", target_type, error) ||
            !read_header(input, "Length Units", length_units, error) ||
            !read_header(input, "Speed Units", speed_units, error) ||
            !read_header(input, "Altitude", altitude_value, error) ||
            !read_header(input, "Speed", speed_value, error)) {
            return false;
        }

        launch_pk_crdr_table table;
        std::string line;
        while (std::getline(input, line)) {
            trim(line);
            if (!line.empty()) break;
        }

        if (line.empty()) {
            if (error) *error = "missing cross-range header row";
            return false;
        }

        {
            std::istringstream iss(line);
            double value = 0.0;
            while (iss >> value) {
                table.cross_ranges_m.push_back(convert_length(value, length_units));
            }
        }

        if (table.cross_ranges_m.size() < 2) {
            if (error) *error = "cross-range row must contain at least two values";
            return false;
        }

        while (std::getline(input, line)) {
            trim(line);
            if (line.empty()) continue;

            std::istringstream iss(line);
            double down_range = 0.0;
            if (!(iss >> down_range)) continue;

            std::vector<double> row;
            row.reserve(table.cross_ranges_m.size());
            double pk = 0.0;
            while (iss >> pk) {
                row.push_back(pk);
            }

            if (row.size() != table.cross_ranges_m.size()) {
                if (error) *error = "pk row width does not match cross-range count";
                return false;
            }

            table.down_ranges_m.push_back(convert_length(down_range, length_units));
            table.pk_grid.push_back(std::move(row));
        }

        if (table.down_ranges_m.size() < 2) {
            if (error) *error = "down-range rows must contain at least two samples";
            return false;
        }

        auto& entries = tables_[std::make_pair(launcher_type, target_type)];
        entries.push_back({
            convert_length(altitude_value, length_units),
            convert_speed(speed_value, speed_units),
            std::move(table)
        });
        return true;
    }

    // 根据请求查询杀伤概率（Evaluate Pk for a given request）
    double evaluate(const launch_pk_request& request) const {
        auto it = tables_.find(std::make_pair(request.launcher_type, request.target_type));
        if (it == tables_.end() || it->second.empty()) return default_pk;

        const auto& entries = it->second;
        std::vector<double> altitudes;
        altitudes.reserve(entries.size());
        for (const auto& entry : entries) altitudes.push_back(entry.altitude_m);
        std::sort(altitudes.begin(), altitudes.end());
        altitudes.erase(std::unique(altitudes.begin(), altitudes.end()), altitudes.end());

        if (altitudes.size() == 1) {
            return evaluate_at_altitude(entries, altitudes.front(), request);
        }

        if (request.altitude_m <= altitudes.front()) {
            return evaluate_at_altitude(entries, altitudes.front(), request);
        }
        if (request.altitude_m >= altitudes.back()) {
            return evaluate_at_altitude(entries, altitudes.back(), request);
        }

        auto upper = std::lower_bound(altitudes.begin(), altitudes.end(), request.altitude_m);
        std::size_t hi = static_cast<std::size_t>(upper - altitudes.begin());
        std::size_t lo = hi - 1;

        double v0 = evaluate_at_altitude(entries, altitudes[lo], request);
        double v1 = evaluate_at_altitude(entries, altitudes[hi], request);
        double t  = (request.altitude_m - altitudes[lo]) / (altitudes[hi] - altitudes[lo]);
        return lerp(v0, v1, t);
    }

private:
    using key_type = std::pair<std::string, std::string>;
    std::map<key_type, std::vector<launch_pk_entry>> tables_;

    // 去除字符串首尾空白（Trim whitespace from string）
    static void trim(std::string& text) {
        auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
        text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
        text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
    }

    // 从文件头读取键值对（Read key-value pair from file header）
    template<typename T>
    static bool read_header(std::ifstream& input,
                            const std::string& expected_name,
                            T& value,
                            std::string* error) {
        std::string key;
        if (!std::getline(input, key, ':')) {
            if (error) *error = "missing header: " + expected_name;
            return false;
        }

        std::string raw_value;
        if (!std::getline(input, raw_value)) {
            if (error) *error = "missing header value: " + expected_name;
            return false;
        }

        trim(key);
        trim(raw_value);
        if (key != expected_name) {
            if (error) *error = "unexpected header: " + key;
            return false;
        }

        std::istringstream iss(raw_value);
        iss >> value;
        if (iss.fail()) {
            if (error) *error = "invalid value for header: " + expected_name;
            return false;
        }
        return true;
    }

    // 字符串类型特化：读取文件头（String overload: read header）
    static bool read_header(std::ifstream& input,
                            const std::string& expected_name,
                            std::string& value,
                            std::string* error) {
        std::string key;
        if (!std::getline(input, key, ':')) {
            if (error) *error = "missing header: " + expected_name;
            return false;
        }

        if (!std::getline(input, value)) {
            if (error) *error = "missing header value: " + expected_name;
            return false;
        }

        trim(key);
        trim(value);
        if (key != expected_name) {
            if (error) *error = "unexpected header: " + key;
            return false;
        }
        return true;
    }

    // 长度单位转换（Length unit conversion）
    static double convert_length(double value, const std::string& units) {
        std::string u = lowercase(units);
        if (u == "m" || u == "meter" || u == "meters") return value;
        if (u == "km" || u == "kilometer" || u == "kilometers") return value * 1000.0;
        if (u == "ft" || u == "feet") return value * 0.3048;
        if (u == "nm" || u == "nmi") return value * 1852.0;
        return value;
    }

    // 速度单位转换（Speed unit conversion）
    static double convert_speed(double value, const std::string& units) {
        std::string u = lowercase(units);
        if (u == "m/s" || u == "mps" || u == "meters/second") return value;
        if (u == "kt" || u == "kts" || u == "knot" || u == "knots") return value * 1852.0 / 3600.0;
        if (u == "km/h" || u == "kph") return value / 3.6;
        return value;
    }

    // 转换为小写（Convert string to lowercase）
    static std::string lowercase(std::string text) {
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return text;
    }

    // 在指定高度下按速度插值评估（Evaluate at given altitude, interpolating over speed）
    static double evaluate_at_speed(const std::vector<launch_pk_entry>& entries,
                                    double altitude_m,
                                    const launch_pk_request& request) {
        std::vector<const launch_pk_entry*> filtered;
        for (const auto& entry : entries) {
            if (entry.altitude_m == altitude_m) filtered.push_back(&entry);
        }
        if (filtered.empty()) return 0.0;

        std::sort(filtered.begin(), filtered.end(), [](const launch_pk_entry* lhs, const launch_pk_entry* rhs) {
            return lhs->target_speed_mps < rhs->target_speed_mps;
        });

        if (filtered.size() == 1) {
            return filtered.front()->table.evaluate(request.down_range_m, request.cross_range_m);
        }

        if (request.target_speed_mps <= filtered.front()->target_speed_mps) {
            return filtered.front()->table.evaluate(request.down_range_m, request.cross_range_m);
        }
        if (request.target_speed_mps >= filtered.back()->target_speed_mps) {
            return filtered.back()->table.evaluate(request.down_range_m, request.cross_range_m);
        }

        std::size_t hi = 1;
        while (hi < filtered.size() && filtered[hi]->target_speed_mps < request.target_speed_mps) ++hi;
        std::size_t lo = hi - 1;

        double v0 = filtered[lo]->table.evaluate(request.down_range_m, request.cross_range_m);
        double v1 = filtered[hi]->table.evaluate(request.down_range_m, request.cross_range_m);
        double t  = (request.target_speed_mps - filtered[lo]->target_speed_mps) /
                   (filtered[hi]->target_speed_mps - filtered[lo]->target_speed_mps);
        return lerp(v0, v1, t);
    }

    // 在指定高度下评估（Evaluate at given altitude）
    static double evaluate_at_altitude(const std::vector<launch_pk_entry>& entries,
                                       double altitude_m,
                                       const launch_pk_request& request) {
        return evaluate_at_speed(entries, altitude_m, request);
    }
};

} // namespace xsf_math
