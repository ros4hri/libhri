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
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)
pybind11_add_module(${PROJECT_NAME} SHARED
  src/pyhri/ndarray_converter.cpp
  src/pyhri/pyhri.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE src/)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_dependencies})
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
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
