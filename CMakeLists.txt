cmake_minimum_required(VERSION 3.0.2)
project(hri)

## Compile as C++17 because of std::optional, even though ROS noetic official only supports c++14
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hri_msgs
  rosconsole
  roscpp
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    #${Boost_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hri
  CATKIN_DEPENDS geometry_msgs hri_msgs rosconsole roscpp std_msgs tf
)

add_library(${PROJECT_NAME}
  src/hri.cpp
)

target_link_libraries(${PROJECT_NAME}   ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

