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
install(DIRECTORY include/xsf_math DESTINATION include)
install(EXPORT xsf_math-targets
    NAMESPACE xsf_math::
    DESTINATION lib/cmake/xsf_math
)
