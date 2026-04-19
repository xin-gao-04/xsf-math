#pragma once

// 编译期日志级别配置（Compile-Time Log Level Configuration）
// 必须先于任何 spdlog 头文件包含，以便提前设置 SPDLOG_ACTIVE_LEVEL（Must be included before any spdlog headers to set SPDLOG_ACTIVE_LEVEL in advance）

#ifndef XSF_ACTIVE_LOG_LEVEL
#  ifdef NDEBUG
     // Release 模式：默认 info 级别（Release mode: default info level, outputs info and above）
#    define XSF_ACTIVE_LOG_LEVEL 2
#  else
     // Debug 模式：默认 debug 级别（Debug mode: default debug level, outputs debug and above）
#    define XSF_ACTIVE_LOG_LEVEL 1
#  endif
#endif

#ifndef SPDLOG_ACTIVE_LEVEL
// 将 XSF 日志级别同步到 spdlog 的编译期宏（Synchronizes XSF log level to spdlog compile-time macro）
#  define SPDLOG_ACTIVE_LEVEL XSF_ACTIVE_LOG_LEVEL
#endif
