#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace xsf {
namespace validation {

struct cli_options {
    bool strict = false;
    bool emit_artifacts = false;
    std::string output_root;
};

struct field {
    std::string key;
    std::string value;
};

inline field make_field(std::string key, std::string value) {
    return {std::move(key), std::move(value)};
}

inline field make_field(std::string key, const char* value) {
    return {std::move(key), value == nullptr ? std::string{} : std::string(value)};
}

inline field make_field(std::string key, bool value) {
    return {std::move(key), value ? "true" : "false"};
}

inline field make_field(std::string key, int value) {
    return {std::move(key), std::to_string(value)};
}

inline field make_field(std::string key, std::size_t value) {
    return {std::move(key), std::to_string(value)};
}

inline field make_field(std::string key, double value, int precision = 6) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return {std::move(key), oss.str()};
}

inline cli_options parse_cli(int argc, char** argv) {
    cli_options options;
    if (const char* env = std::getenv("XSF_VALIDATION_OUTPUT_DIR")) {
        if (*env != '\0') {
            options.output_root = env;
            options.emit_artifacts = true;
        }
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--strict") {
            options.strict = true;
        } else if (arg == "--output-dir") {
            if (i + 1 < argc) {
                options.output_root = argv[++i];
                options.emit_artifacts = true;
            }
        } else if (arg == "--artifacts") {
            options.emit_artifacts = true;
        }
    }

    return options;
}

inline std::string csv_escape(const std::string& text) {
    bool needs_quotes = false;
    for (char ch : text) {
        if (ch == ',' || ch == '"' || ch == '\n' || ch == '\r') {
            needs_quotes = true;
            break;
        }
    }
    if (!needs_quotes) return text;

    std::string escaped;
    escaped.reserve(text.size() + 4);
    escaped.push_back('"');
    for (char ch : text) {
        if (ch == '"') escaped.push_back('"');
        escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

inline std::string json_escape(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size() + 8);
    for (char ch : text) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

class case_artifacts {
public:
    case_artifacts(std::string case_id, const cli_options& options)
        : case_id_(std::move(case_id)),
          enabled_(options.emit_artifacts || !options.output_root.empty()) {
        if (!enabled_) return;

        std::filesystem::path root = options.output_root.empty()
            ? std::filesystem::current_path()
            : std::filesystem::path(options.output_root);
        case_dir_ = root / case_id_;
        std::filesystem::create_directories(case_dir_);
    }

    bool enabled() const noexcept { return enabled_; }

    const std::filesystem::path& case_dir() const noexcept { return case_dir_; }

    void write_timeseries_header(const std::vector<std::string>& columns) {
        if (!enabled_) return;
        timeseries_.open(case_dir_ / "timeseries.csv", std::ios::out | std::ios::trunc);
        write_csv_row(timeseries_, columns);
    }

    void append_timeseries_row(const std::vector<std::string>& values) {
        if (!enabled_) return;
        if (!timeseries_.is_open()) return;
        write_csv_row(timeseries_, values);
    }

    void append_event(double time_s,
                      const std::string& event_type,
                      const std::string& message,
                      const std::vector<field>& attributes = {}) {
        if (!enabled_) return;
        if (!events_.is_open()) {
            events_.open(case_dir_ / "events.jsonl", std::ios::out | std::ios::trunc);
        }

        events_ << "{"
                << "\"time_s\":\"" << json_escape(make_field("", time_s, 6).value) << "\","
                << "\"event_type\":\"" << json_escape(event_type) << "\","
                << "\"message\":\"" << json_escape(message) << "\","
                << "\"attributes\":{";
        for (std::size_t i = 0; i < attributes.size(); ++i) {
            if (i != 0) events_ << ",";
            events_ << "\"" << json_escape(attributes[i].key) << "\":"
                    << "\"" << json_escape(attributes[i].value) << "\"";
        }
        events_ << "}}\n";
    }

    void write_metrics(const std::vector<field>& metrics) const {
        if (!enabled_) return;
        write_object_file(case_dir_ / "metrics.json", metrics);
    }

    void write_summary(bool passed,
                       const std::string& purpose,
                       const std::string& expected_behavior,
                       const std::string& conclusion,
                       const std::vector<field>& inputs,
                       const std::vector<field>& metrics,
                       const std::string& failure_reason = std::string()) const {
        if (!enabled_) return;

        std::ofstream output(case_dir_ / "run_summary.json", std::ios::out | std::ios::trunc);
        output << "{\n";
        output << "  \"case_id\": \"" << json_escape(case_id_) << "\",\n";
        output << "  \"passed\": " << (passed ? "true" : "false") << ",\n";
        output << "  \"purpose\": \"" << json_escape(purpose) << "\",\n";
        output << "  \"expected_behavior\": \"" << json_escape(expected_behavior) << "\",\n";
        output << "  \"conclusion\": \"" << json_escape(conclusion) << "\",\n";
        output << "  \"failure_reason\": \"" << json_escape(failure_reason) << "\",\n";
        output << "  \"inputs\": ";
        write_object(output, inputs, 2);
        output << ",\n";
        output << "  \"metrics\": ";
        write_object(output, metrics, 2);
        output << "\n}\n";
    }

private:
    static void write_csv_row(std::ofstream& stream, const std::vector<std::string>& values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) stream << ",";
            stream << csv_escape(values[i]);
        }
        stream << "\n";
    }

    static void write_object_file(const std::filesystem::path& path, const std::vector<field>& fields) {
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        write_object(output, fields, 0);
        output << "\n";
    }

    static void write_object(std::ofstream& output, const std::vector<field>& fields, int indent) {
        output << "{";
        if (!fields.empty()) output << "\n";
        for (std::size_t i = 0; i < fields.size(); ++i) {
            output << std::string(indent + 2, ' ')
                   << "\"" << json_escape(fields[i].key) << "\": "
                   << "\"" << json_escape(fields[i].value) << "\"";
            if (i + 1 != fields.size()) output << ",";
            output << "\n";
        }
        if (!fields.empty()) output << std::string(indent, ' ');
        output << "}";
    }

    std::string case_id_;
    bool enabled_ = false;
    std::filesystem::path case_dir_;
    std::ofstream timeseries_;
    std::ofstream events_;
};

} // namespace validation
} // namespace xsf
