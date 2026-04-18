add_executable(test_core test_core.cpp)
target_link_libraries(test_core PRIVATE xsf_math)
add_test(NAME test_core COMMAND test_core)

add_executable(test_radar test_radar.cpp)
target_link_libraries(test_radar PRIVATE xsf_math)
add_test(NAME test_radar COMMAND test_radar)

add_executable(test_guidance test_guidance.cpp)
target_link_libraries(test_guidance PRIVATE xsf_math)
add_test(NAME test_guidance COMMAND test_guidance)

add_executable(test_guidance_programs test_guidance_programs.cpp)
target_link_libraries(test_guidance_programs PRIVATE xsf_math)
add_test(NAME test_guidance_programs COMMAND test_guidance_programs)

add_executable(test_flight test_flight.cpp)
target_link_libraries(test_flight PRIVATE xsf_math)
add_test(NAME test_flight COMMAND test_flight)

add_executable(test_algorithm_io test_algorithm_io.cpp)
target_link_libraries(test_algorithm_io PRIVATE xsf_math)
target_compile_definitions(test_algorithm_io PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
add_test(NAME test_algorithm_io COMMAND test_algorithm_io)

add_executable(test_log test_log.cpp)
target_link_libraries(test_log PRIVATE xsf_math)
add_test(NAME test_log COMMAND test_log)

add_executable(test_tracking_identity test_tracking_identity.cpp)
target_link_libraries(test_tracking_identity PRIVATE xsf_math)
add_test(NAME test_tracking_identity COMMAND test_tracking_identity)

add_executable(test_roadmap_features test_roadmap_features.cpp)
target_link_libraries(test_roadmap_features PRIVATE xsf_math)
add_test(NAME test_roadmap_features COMMAND test_roadmap_features)
