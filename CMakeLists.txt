cmake_minimum_required(VERSION 3.0.2)
project(hri)

## ROS noetic officially supports c++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hri_msgs
  sensor_msgs
  rosconsole
  roscpp
  std_msgs
  tf2
  tf2_ros
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED) # for boost::optional

include_directories(
    SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

include_directories(
    include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hri
  CATKIN_DEPENDS geometry_msgs hri_msgs rosconsole roscpp std_msgs tf2 cv_bridge
)

add_library(${PROJECT_NAME}
  src/hri.cpp
  src/body.cpp
  src/face.cpp
  src/person.cpp
  src/voice.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

#find_package(catkin REQUIRED COMPONENTS
#  tf2 tf2_ros)

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

