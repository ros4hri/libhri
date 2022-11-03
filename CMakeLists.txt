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
  src/libhri/hri.cpp
  src/libhri/body.cpp
  src/libhri/face.cpp
  src/libhri/person.cpp
  src/libhri/voice.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(hri src/hri.cpp)
ament_target_dependencies(hri ${dependencies})
target_link_libraries(body ${PROJECT_NAME})

add_executable(body src/body.cpp)
ament_target_dependencies(body ${dependencies})
target_link_libraries(hri ${PROJECT_NAME})

add_executable(hri src/hri.cpp)
ament_target_dependencies(hri ${dependencies})
target_link_libraries(hri ${PROJECT_NAME})

add_executable(hri src/hri.cpp)
ament_target_dependencies(hri ${dependencies})
target_link_libraries(hri ${PROJECT_NAME})


if(BUILD_EXAMPLES)
add_executable(show_faces_example
    examples/show_faces.cpp
)
target_link_libraries(show_faces_example ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()

###########################################################
# Tests

if(CATKIN_ENABLE_TESTING)

find_package(rostest REQUIRED)

add_rostest_gmock(test_hri test/test_hri.test test/test_hri.cpp)
target_include_directories(test_hri PRIVATE ${rostest_INCLUDE_DIRS})
target_link_libraries(test_hri ${PROJECT_NAME})

endif() # CATKIN_ENABLE_TESTING


###########################################################
# Install

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

