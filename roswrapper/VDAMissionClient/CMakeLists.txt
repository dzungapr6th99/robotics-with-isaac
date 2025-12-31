cmake_minimum_required(VERSION 3.8)
project(VDAMissionClient)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(example_handler_dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  vda5050_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_geometry_msgs
  lifecycle_msgs
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vda5050_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# Build share library
add_library(VDAMissionClient SHARED
  src/VDAMissionClient.cpp
  src/OrderWrapper.cpp
  src/InstantActionsWrapper.cpp
  src/NodeWrapper.cpp
  src/EdgeWrapper.cpp
  src/ActionWrapper.cpp
  src/NodePositionWrapper.cpp
  src/ActionParameterWrapper.cpp
  src/ControlPointWrapper.cpp
  src/TrajectoryWrapper.cpp
  src/FactsheetWrapper.cpp
  src/AGVStateWrapper.cpp
  src/VisualizationWrapper.cpp
  src/AGVPositionWrapper.cpp
  src/VelocityWrapper.cpp
  src/TypeSpecificationWrapper.cpp
  src/PhysicalParametersWrapper.cpp
  src/BatteryStateWrapper.cpp
  src/SafetyStateWrapper.cpp
  src/NodeStateWrapper.cpp
  src/EdgeStateWrapper.cpp
  src/ActionStateWrapper.cpp
)

add_custom_command(
  TARGET VDAMissionClient
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          $<TARGET_FILE:VDAMissionClient>
          $<TARGET_FILE_DIR:VDAMissionClient>/libVDAWrapper.so
)

ament_target_dependencies(VDAMissionClient rclcpp geometry_msgs nav2_msgs lifecycle_msgs rclcpp_action vda5050_msgs sensor_msgs tf2_geometry_msgs tf2 pluginlib)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
