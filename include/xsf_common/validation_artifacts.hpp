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

// 命令行选项（Command-Line Options）
// 解析验证测试的 CLI 参数：严格模式、产物输出、输出目录（Parses CLI arguments for validation tests: strict mode, artifact output, output directory）
struct cli_options {
    bool strict = false;             // 严格模式（Strict Mode）：任何失败即终止
    bool emit_artifacts = false;     // 是否输出验证产物（Emit Artifacts）
    std::string output_root;         // 输出根目录（Output Root Directory）
};

// 字段键值对（Field Key-Value Pair）
// 用于构造结构化日志条目（Used to construct structured log entries: Metrics / Summary / Events）
struct field {
    std::string key;                 // 字段键名（Field Key）
    std::string value;               // 字段值（Field Value）
};

// 构造字段工厂函数（Field Factory Functions）
// 支持多种数据类型的自动转换（Supports automatic conversion of multiple data types）
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

// 解析命令行参数（Parse Command-Line Arguments）
// 支持环境变量 XSF_VALIDATION_OUTPUT_DIR 和命令行 --strict / --output-dir / --artifacts（Supports XSF_VALIDATION_OUTPUT_DIR environment variable and --strict / --output-dir / --artifacts command-line flags）
inline cli_options parse_cli(int argc, char** argv) {
    cli_options options;
    // 优先检查环境变量（Check environment variable first）
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

// CSV 字段转义（CSV Field Escape）
// 对包含逗号、引号、换行符的字段进行 RFC 4180 兼容转义（RFC 4180 compliant escaping for fields containing commas, quotes, or newlines）
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
        if (ch == '"') escaped.push_back('"');  // 双引号转义（Double quote escape）：" -> ""
        escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

// JSON 字符串转义（JSON String Escape）
// 对需要写入 JSON 的字符串进行标准转义（Standard escaping for strings to be written to JSON）
inline std::string json_escape(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size() + 8);
    for (char ch : text) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;  // 反斜杠（Backslash）
            case '"': escaped += "\\\""; break;  // 双引号（Double Quote）
            case '\n': escaped += "\\n"; break;   // 换行（Newline）
            case '\r': escaped += "\\r"; break;   // 回车（Carriage Return）
            case '\t': escaped += "\\t"; break;   // 制表符（Tab）
            default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

// 验证测试用例产物收集器（Validation Case Artifacts Collector）
// 为单个测试用例收集时序数据、事件日志和汇总报告（Collects timeseries data, event logs, and summary reports for a single test case）
// 输出格式：CSV（时序）、JSONL（事件）、JSON（汇总/指标）（Output formats: CSV (timeseries), JSONL (events), JSON (summary/metrics)）
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

    // 是否启用产物输出（Is Artifact Output Enabled）
    bool enabled() const noexcept { return enabled_; }

    // 获取用例输出目录（Get Case Output Directory）
    const std::filesystem::path& case_dir() const noexcept { return case_dir_; }

    // 写入时序 CSV 表头（Write Timeseries CSV Header）
    void write_timeseries_header(const std::vector<std::string>& columns) {
        if (!enabled_) return;
        timeseries_.open(case_dir_ / "timeseries.csv", std::ios::out | std::ios::trunc);
        write_csv_row(timeseries_, columns);
    }

    // 追加时序 CSV 数据行（Append Timeseries CSV Row）
    void append_timeseries_row(const std::vector<std::string>& values) {
        if (!enabled_) return;
        if (!timeseries_.is_open()) return;
        write_csv_row(timeseries_, values);
    }

    // 追加事件日志（Append Event Log）
    // 输出格式：JSON Lines（JSONL），每行一个 JSON 对象（Output format: JSON Lines (JSONL), one JSON object per line）
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

    // 写入指标文件（Write Metrics File）
    // 输出：metrics.json（Output: metrics.json）
    void write_metrics(const std::vector<field>& metrics) const {
        if (!enabled_) return;
        write_object_file(case_dir_ / "metrics.json", metrics);
    }

    // 写入运行汇总（Write Run Summary）
    // 输出：run_summary.json，包含用例 ID、通过状态、目的、结论、输入参数、指标（Output: run_summary.json, containing case ID, pass status, purpose, conclusion, inputs, and metrics）
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
    // 写入 CSV 行（Write CSV Row）
    static void write_csv_row(std::ofstream& stream, const std::vector<std::string>& values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) stream << ",";
            stream << csv_escape(values[i]);
        }
        stream << "\n";
    }

    // 写入 JSON 对象文件（Write JSON Object File）
    static void write_object_file(const std::filesystem::path& path, const std::vector<field>& fields) {
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        write_object(output, fields, 0);
        output << "\n";
    }

    // 写入 JSON 对象（Write JSON Object）
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

    std::string case_id_;            // 用例 ID（Case ID）
    bool enabled_ = false;           // 是否启用（Enabled Flag）
    std::filesystem::path case_dir_; // 用例输出目录（Case Output Directory）
    std::ofstream timeseries_;       // 时序数据输出流（Timeseries Output Stream）
    std::ofstream events_;           // 事件日志输出流（Events Output Stream）
};

} // namespace validation
} // namespace xsf
