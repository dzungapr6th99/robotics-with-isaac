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
set(vda_wrapper_sources
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
  src/ErrorWrapper.cpp
  src/LoadWrapper.cpp
  src/InfoWrapper.cpp
)

set(vda_mission_sources
  src/VDAMissionClient.cpp
)

set(vda_action_handler_sources
  src/VDAActionAMRHandler.cpp
)

# Build shared libraries
add_library(VDAWrapper SHARED ${vda_wrapper_sources})
add_library(VDAMissionClient SHARED ${vda_mission_sources})
add_library(vda_action_amr_handler SHARED ${vda_action_handler_sources})

ament_target_dependencies(VDAWrapper rclcpp geometry_msgs nav2_msgs lifecycle_msgs rclcpp_action vda5050_msgs sensor_msgs tf2_geometry_msgs tf2 pluginlib)
ament_target_dependencies(VDAMissionClient rclcpp geometry_msgs nav2_msgs lifecycle_msgs rclcpp_action vda5050_msgs sensor_msgs tf2_geometry_msgs tf2 pluginlib)
ament_target_dependencies(
  vda_action_amr_handler
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  lifecycle_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  vda5050_msgs
  pluginlib
)
target_link_libraries(VDAMissionClient VDAWrapper)

pluginlib_export_plugin_description_file(${PROJECT_NAME} vda_action_handlers.xml)

install(
  TARGETS
    VDAWrapper
    VDAMissionClient
    vda_action_amr_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES vda_action_handlers.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(pluginlib)
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
