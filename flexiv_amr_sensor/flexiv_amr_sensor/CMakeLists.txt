cmake_minimum_required(VERSION 3.8)
project(flexiv_amr_sensor)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  flexiv_amr_msgs
  flexiv_amr_sdk
  rclcpp
  sensor_msgs
  std_msgs
)

ament_auto_add_executable(flexiv_amr_sensor src/flexiv_amr_sensor_node.cpp)
target_link_libraries(flexiv_amr_sensor flexiv::flexiv_amr)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch config
  USE_SCOPED_HEADER_INSTALL_DIR
)
