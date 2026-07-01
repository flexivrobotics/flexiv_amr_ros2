cmake_minimum_required(VERSION 3.8)
project(flexiv_amr_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  flexiv_amr_msgs
  flexiv_amr_sdk
  rclcpp
)

ament_auto_add_executable(flexiv_amr_navigation_node src/flexiv_amr_navigation_node.cpp)
target_link_libraries(flexiv_amr_navigation_node flexiv::amr)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Keep ROS interface and launch packaging checks enabled; use clang-format for C++ style.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
