cmake_minimum_required(VERSION 3.14)
project(xsf_math
    VERSION 0.1.0
    DESCRIPTION "Header-only math/computation library extracted from XSF-Core"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

# Header-only library
add_library(xsf_math INTERFACE)
target_include_directories(xsf_math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Logging backend (optional, defaults to header-only spdlog) ───────────────
option(XSF_ENABLE_LOGGING "Enable spdlog-based logging (requires spdlog)" ON)
set(XSF_LOGGING_PROVIDER "auto" CACHE STRING
    "Logging backend provider: auto, system, fetch, off")
set_property(CACHE XSF_LOGGING_PROVIDER PROPERTY STRINGS auto bundled system fetch off)
set(XSF_SPDLOG_GIT_TAG "v1.17.0" CACHE STRING
    "spdlog tag used when XSF_LOGGING_PROVIDER fetches it automatically")

if(XSF_ENABLE_LOGGING)
    set(_xsf_logging_enabled FALSE)
    set(_xsf_logging_provider "")

    if(XSF_LOGGING_PROVIDER STREQUAL "off")
        set(XSF_ENABLE_LOGGING OFF CACHE BOOL "" FORCE)
    endif()

    # 1) bundled (preferred when explicitly requested or when auto-detected)
    if(XSF_ENABLE_LOGGING AND (XSF_LOGGING_PROVIDER STREQUAL "bundled"
       OR (XSF_LOGGING_PROVIDER STREQUAL "auto" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include/spdlog/spdlog.h")))
        set(_xsf_spdlog_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/spdlog/include")
        if(EXISTS "${_xsf_spdlog_include_dir}/spdlog/spdlog.h")
            set(_xsf_logging_enabled TRUE)
            set(_xsf_logging_provider "bundled")
        elseif(XSF_LOGGING_PROVIDER STREQUAL "bundled")
            message(FATAL_ERROR
                "xsf-math: XSF_LOGGING_PROVIDER=bundled but third_party/spdlog/include/spdlog/spdlog.h not found.")
        endif()
    endif()

    # 2) system package
    if(XSF_ENABLE_LOGGING AND NOT _xsf_logging_enabled AND NOT XSF_LOGGING_PROVIDER STREQUAL "fetch")
        find_package(spdlog CONFIG QUIET)
        if(spdlog_FOUND)
            set(_xsf_logging_enabled TRUE)
            set(_xsf_logging_provider "system")
        elseif(XSF_LOGGING_PROVIDER STREQUAL "system")
            message(FATAL_ERROR
                "xsf-math: XSF_LOGGING_PROVIDER=system but spdlog was not found. "
                "Provide spdlog via your package manager or switch to auto/fetch/bundled.")
        endif()
    endif()

    if(XSF_ENABLE_LOGGING AND NOT _xsf_logging_enabled
       AND (XSF_LOGGING_PROVIDER STREQUAL "auto" OR XSF_LOGGING_PROVIDER STREQUAL "fetch"))
        include(FetchContent)
        set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_EXAMPLE_HO OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_TESTS_HO OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
        set(SPDLOG_BUILD_WARNINGS OFF CACHE BOOL "" FORCE)
        set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
        set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
        set(SPDLOG_FMT_EXTERNAL_HO OFF CACHE BOOL "" FORCE)

        FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG        ${XSF_SPDLOG_GIT_TAG}
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(spdlog)
        set(_xsf_spdlog_include_dir "${spdlog_SOURCE_DIR}/include")
        set(_xsf_logging_enabled TRUE)
        set(_xsf_logging_provider "fetch")
    endif()

    if(_xsf_logging_enabled)
        if(_xsf_logging_provider STREQUAL "fetch" OR _xsf_logging_provider STREQUAL "bundled")
            target_include_directories(xsf_math INTERFACE
                $<BUILD_INTERFACE:${_xsf_spdlog_include_dir}>
            )
        elseif(TARGET spdlog::spdlog_header_only)
            target_link_libraries(xsf_math INTERFACE spdlog::spdlog_header_only)
        elseif(TARGET spdlog_header_only)
            target_link_libraries(xsf_math INTERFACE spdlog_header_only)
        elseif(TARGET spdlog::spdlog)
            target_link_libraries(xsf_math INTERFACE spdlog::spdlog)
        elseif(TARGET spdlog)
            target_link_libraries(xsf_math INTERFACE spdlog)
        else()
            message(FATAL_ERROR "xsf-math: spdlog was resolved but no usable header-only target was found.")
        endif()

        target_compile_definitions(xsf_math INTERFACE XSF_ENABLE_LOGGING)
        message(STATUS "xsf-math: logging enabled via ${_xsf_logging_provider} spdlog (${XSF_SPDLOG_GIT_TAG})")
    else()
        message(STATUS
            "xsf-math: spdlog unavailable, logging disabled. "
            "Use XSF_LOGGING_PROVIDER=fetch for automatic header-only download, "
            "provide a system package, or set XSF_ENABLE_LOGGING=OFF.")
        set(XSF_ENABLE_LOGGING OFF CACHE BOOL "" FORCE)
    endif()
endif()

# Examples
option(XSF_MATH_BUILD_EXAMPLES "Build examples" ON)
if(XSF_MATH_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
option(XSF_MATH_BUILD_TESTS "Build tests" ON)
if(XSF_MATH_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install
install(TARGETS xsf_math EXPORT xsf_math-targets)
install(DIRECTORY include/xsf_common DESTINATION include)
install(DIRECTORY include/xsf_math  DESTINATION include)
install(DIRECTORY include/xsf_behavior DESTINATION include)
if(DEFINED _xsf_spdlog_include_dir AND EXISTS "${_xsf_spdlog_include_dir}/spdlog")
    install(DIRECTORY "${_xsf_spdlog_include_dir}/spdlog" DESTINATION include)
endif()
install(EXPORT xsf_math-targets
    NAMESPACE xsf_math::
    DESTINATION lib/cmake/xsf_math
)
