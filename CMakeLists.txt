cmake_minimum_required(VERSION 3.28)
project(mrs_usv_controller_plugin)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_usv_controller_plugin]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# Find all dependencies of this package
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_uav_managers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mrs_uav_testing REQUIRED)

## | ------------------- MRS USV Controller ------------------- |
add_library(MrsUsvControllerPlugin_MrsUsvController SHARED)

target_sources(MrsUsvControllerPlugin_MrsUsvController
  PRIVATE
    "src/mrs_usv_controller.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/pid.hpp"
)

target_link_libraries(MrsUsvControllerPlugin_MrsUsvController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
)

target_compile_definitions(MrsUsvControllerPlugin_MrsUsvController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## | ----------------- Passthrough Controller ----------------- |
add_library(MrsUsvControllerPlugin_PassthroughController SHARED)

target_sources(MrsUsvControllerPlugin_PassthroughController
  PRIVATE
    "src/passthrough_controller.cpp"
)

target_link_libraries(MrsUsvControllerPlugin_PassthroughController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
)

## | ------------------- Position Controller ------------------ |
add_library(MrsUsvControllerPlugin_PositionController SHARED)

target_sources(MrsUsvControllerPlugin_PositionController
  PRIVATE
    "src/position_controller.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
)

target_link_libraries(MrsUsvControllerPlugin_PositionController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
)

target_compile_definitions(MrsUsvControllerPlugin_PositionController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS MrsUsvControllerPlugin_MrsUsvController
  TARGETS MrsUsvControllerPlugin_PositionController
  TARGETS MrsUsvControllerPlugin_PassthroughController
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()

