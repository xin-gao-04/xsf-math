#pragma once

#include <xsf_common/log_config.hpp>

#ifdef XSF_ENABLE_LOGGING

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace xsf {
namespace log {

enum class level : int {
    trace = 0,
    debug = 1,
    info = 2,
    warn = 3,
    error = 4,
    critical = 5,
    off = 6
};

namespace detail {

inline spdlog::logger* get() {
    static auto inst = [] {
        auto existing = spdlog::get("xsf");
        if (existing) return existing;
        auto logger = spdlog::stderr_color_mt("xsf");
        logger->set_level(static_cast<spdlog::level::level_enum>(XSF_ACTIVE_LOG_LEVEL));
        logger->set_pattern("[%T.%e] [%^%-8l%$] %v");
        return logger;
    }();
    return inst.get();
}

} // namespace detail

inline void set_level(level l) noexcept {
    detail::get()->set_level(
        static_cast<spdlog::level::level_enum>(static_cast<int>(l)));
}

inline void set_level(int l) noexcept {
    detail::get()->set_level(static_cast<spdlog::level::level_enum>(l));
}

inline void set_pattern(const char* pattern) {
    detail::get()->set_pattern(pattern);
}

} // namespace log
} // namespace xsf

#define XSF_LOG_TRACE(...)    SPDLOG_LOGGER_TRACE(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_INFO(...)     SPDLOG_LOGGER_INFO(    xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_WARN(...)     SPDLOG_LOGGER_WARN(    xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_ERROR(...)    SPDLOG_LOGGER_ERROR(   xsf::log::detail::get(), __VA_ARGS__)
#define XSF_LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(xsf::log::detail::get(), __VA_ARGS__)

#else

namespace xsf {
namespace log {

enum class level : int {
    trace = 0,
    debug = 1,
    info = 2,
    warn = 3,
    error = 4,
    critical = 5,
    off = 6
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
