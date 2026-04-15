#pragma once

// 编译期日志级别配置。
// 必须先于任何 spdlog 头文件包含，以便提前设置 SPDLOG_ACTIVE_LEVEL。

#ifndef XSF_ACTIVE_LOG_LEVEL
#  ifdef NDEBUG
#    define XSF_ACTIVE_LOG_LEVEL 2
#  else
#    define XSF_ACTIVE_LOG_LEVEL 1
#  endif
#endif

#ifndef SPDLOG_ACTIVE_LEVEL
#  define SPDLOG_ACTIVE_LEVEL XSF_ACTIVE_LOG_LEVEL
#endif
