cmake_minimum_required(VERSION 3.5)
project(hri)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

## ROS noetic officially supports c++14
# add_compile_options(-std=c++14)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED) 

set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    hri_msgs
    tf2_ros
    tf2
    cv_bridge
    OpenCV
  )


include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/hri/hri.cpp
  src/hri/body.cpp
  src/hri/face.cpp
  src/hri/person.cpp
  src/hri/voice.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})



add_executable(show_faces examples/show_faces.cpp)
ament_target_dependencies(show_faces ${dependencies})
target_link_libraries(show_faces ${PROJECT_NAME})


install(TARGETS
  show_faces
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

###########################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()


install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


