ament_add_gtest(libhri_tests test_hri.cpp)
ament_target_dependencies(libhri_tests ${dependencies})
target_link_libraries(libhri_tests ${PROJECT_NAME})
