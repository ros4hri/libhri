ament_add_gtest(libhri_tests test_hri.cpp)
ament_target_dependencies(libhri_tests ${dependencies})
target_link_libraries(libhri_tests ${PROJECT_NAME})

ament_add_gmock(libhri_callback_tests test_callback.cpp)
ament_target_dependencies(libhri_callback_tests ${dependencies})
target_link_libraries(libhri_callback_tests ${PROJECT_NAME})
