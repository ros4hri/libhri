cmake_minimum_required(VERSION 3.8)
project(hri)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Depencencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  cv_bridge
  geometry_msgs
  hri_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
ament_export_dependencies(${dependencies})

# Public library target
add_library(${PROJECT_NAME} SHARED
  src/hri/hri.cpp
  src/hri/body.cpp
  src/hri/face.cpp
  src/hri/person.cpp
  src/hri/voice.cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Example target
set(example_name show_faces)
add_executable(${example_name} src/node_show_faces.cpp)
target_link_libraries(${example_name} PRIVATE ${PROJECT_NAME})
install(
  TARGETS ${example_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
