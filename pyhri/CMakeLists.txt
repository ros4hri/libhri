# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(pyhri)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)  # Find python before pybind11
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(hri REQUIRED)
find_package(rclcpp REQUIRED)

set(${PROJECT_NAME}_dependencies
  builtin_interfaces
  geometry_msgs
  hri
  rclcpp
)
ament_export_dependencies(${${PROJECT_NAME}_dependencies})

# Python target
ament_python_install_package(hri
  PACKAGE_DIR hri
  SCRIPTS_DESTINATION lib/hri
)

set(HRI_MODULE_NAME hri)
pybind11_add_module(${HRI_MODULE_NAME} SHARED
  src/pyhri/ndarray_converter.cpp
  src/pyhri/pyhri.cpp)
target_include_directories(${HRI_MODULE_NAME} PRIVATE src/)
ament_target_dependencies(${HRI_MODULE_NAME} PUBLIC ${${PROJECT_NAME}_dependencies})
install(TARGETS ${HRI_MODULE_NAME}
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/hri"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    src/pyhri/ndarray_converter.h
    src/pyhri/ndarray_converter.cpp)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_pyhri test/test_pyhri.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
  )
endif()

ament_package()
