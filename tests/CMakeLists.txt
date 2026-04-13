add_executable(test_core test_core.cpp)
target_link_libraries(test_core PRIVATE xsf_math)
add_test(NAME test_core COMMAND test_core)

add_executable(test_radar test_radar.cpp)
target_link_libraries(test_radar PRIVATE xsf_math)
add_test(NAME test_radar COMMAND test_radar)

add_executable(test_guidance test_guidance.cpp)
target_link_libraries(test_guidance PRIVATE xsf_math)
add_test(NAME test_guidance COMMAND test_guidance)
