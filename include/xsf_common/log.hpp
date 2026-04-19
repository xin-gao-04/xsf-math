#pragma once

#include <xsf_common/log_config.hpp>

#ifdef XSF_ENABLE_LOGGING

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace xsf {
namespace log {

// 日志级别枚举（Log Level Enumeration）
// 与 spdlog::level 对应，用于运行时日志级别控制（Maps to spdlog::level for runtime log level control）
enum class level : int {
    trace = 0,      // 跟踪级（Trace）：最详细的调试信息
    debug = 1,      // 调试级（Debug）：开发调试信息
    info = 2,       // 信息级（Info）：正常运行信息
    warn = 3,       // 警告级（Warn）：潜在问题
    error = 4,      // 错误级（Error）：运行错误
    critical = 5,   // 严重级（Critical）：系统级严重错误
    off = 6         // 关闭（Off）：禁用日志输出
};

namespace detail {

// 获取/创建 spdlog 日志器实例（Get/Create spdlog Logger Instance）
// 使用延迟初始化（Lazy Initialization）的单例模式，默认输出到标准错误（stderr）
inline spdlog::logger* get() {
    static auto inst = [] {
        auto existing = spdlog::get("xsf");
        if (existing) return existing;
        auto logger = spdlog::stderr_color_mt("xsf");
        // 设置编译期日志级别（Set Compile-Time Log Level）
        logger->set_level(static_cast<spdlog::level::level_enum>(XSF_ACTIVE_LOG_LEVEL));
        // 日志格式：时间 [级别] 消息（Log format: time [level] message）
        logger->set_pattern("[%T.%e] [%^%-8l%$] %v");
        return logger;
    }();
    return inst.get();
}

} // namespace detail

// 设置日志级别（Set Log Level）- 枚举版本
inline void set_level(level l) noexcept {
    detail::get()->set_level(
        static_cast<spdlog::level::level_enum>(static_cast<int>(l)));
}

// 设置日志级别（Set Log Level）- 整型版本
inline void set_level(int l) noexcept {
    detail::get()->set_level(static_cast<spdlog::level::level_enum>(l));
}

// 设置日志格式模式（Set Log Pattern）
inline void set_pattern(const char* pattern) {
    detail::get()->set_pattern(pattern);
}

} // namespace log
} // namespace xsf

// XSF 日志宏（XSF Logging Macros）
// 在启用了 XSF_ENABLE_LOGGING 时映射到 spdlog，否则为空操作（Maps to spdlog when XSF_ENABLE_LOGGING is enabled, otherwise no-op）
#define XSF_LOG_TRACE(...)    SPDLOG_LOGGER_TRACE(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_INFO(...)     SPDLOG_LOGGER_INFO(    xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_WARN(...)     SPDLOG_LOGGER_WARN(    xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_ERROR(...)    SPDLOG_LOGGER_ERROR(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(xsf::log::detail::get(), __VA_ARGS__)

#else

// 日志禁用版本（Logging Disabled Version）：所有操作空实现
namespace xsf {
namespace log {

enum class level : int {
    trace = 0,      // 跟踪级（Trace）
    debug = 1,      // 调试级（Debug）
    info = 2,       // 信息级（Info）
    warn = 3,       // 警告级（Warn）
    error = 4,      // 错误级（Error）
    critical = 5,   // 严重级（Critical）
    off = 6         // 关闭（Off）
};

inline void set_level(level) noexcept {}
inline void set_level(int) noexcept {}
inline void set_pattern(const char*) noexcept {}

} // namespace log
} // namespace xsf

#define XSF_LOG_TRACE(...)    (void)0
#define XSF_LOG_DEBUG(...)    (void)0
#define XSF_LOG_INFO(...)     (void)0
#define XSF_LOG_WARN(...)     (void)0
#define XSF_LOG_ERROR(...)    (void)0
#define XSF_LOG_CRITICAL(...) (void)0

#endif
