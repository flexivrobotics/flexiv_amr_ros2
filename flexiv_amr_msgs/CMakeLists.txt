cmake_minimum_required(VERSION 3.8)
project(flexiv_amr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AmrBattery.msg"
  "msg/AmrBlockStatus.msg"
  "msg/AmrEncoder.msg"
  "msg/AmrImu.msg"
  "msg/AmrLocalMotion.msg"
  "msg/AmrInfo.msg"
  "msg/AmrPose.msg"
  "msg/AmrRunInfo.msg"
  "msg/AmrSpeed.msg"
  "msg/AmrUltrasonic.msg"
  "msg/AmrUltrasonicNode.msg"
  "srv/AmrCancelNavigation.srv"
  "srv/AmrCancelRelocation.srv"
  "srv/AmrEmergencyStop.srv"
  "srv/AmrFixedPathNavigation.srv"
  "srv/AmrLocationStatus.srv"
  "srv/AmrNavigationStatus.srv"
  "srv/AmrPauseNavigation.srv"
  "srv/AmrRelocation.srv"
  "srv/AmrRotation.srv"
  "srv/AmrResumeNavigation.srv"
  "srv/AmrSpecifiedPathNavigation.srv"
  "srv/AmrStopOpenLoopMotion.srv"
  "srv/AmrTaskStatus.srv"
  "srv/AmrTranslation.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ROS interface definition files are validated by rosidl and do not carry source headers.
  set(ament_cmake_copyright_FOUND TRUE)
  # C++ style checks are not applicable to this interface-only package.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
