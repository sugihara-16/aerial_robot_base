cmake_minimum_required(VERSION 3.5)
project(gimbalrotor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aerial_robot_control REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_navigation REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(spinal_msgs REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  pluginlib
  angles
  tf2
  tf2_geometry_msgs
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_navigation
  aerial_robot_msgs
  spinal
  spinal_msgs
)

add_library(gimbalrotor_robot_model SHARED
  src/model/gimbalrotor_robot_model.cpp
)
ament_target_dependencies(gimbalrotor_robot_model
  ${ROS_DEPENDENCIES}
)
target_include_directories(gimbalrotor_robot_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(gimbalrotor_robot_model
  Eigen3::Eigen
)

add_library(gimbalrotor_controller SHARED
  src/control/gimbalrotor_controller.cpp
)
ament_target_dependencies(gimbalrotor_controller
  ${ROS_DEPENDENCIES}
)
target_include_directories(gimbalrotor_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(gimbalrotor_controller
  gimbalrotor_robot_model
  Eigen3::Eigen
)

add_library(gimbalrotor_navigation SHARED
  src/gimbalrotor_navigation.cpp
)
ament_target_dependencies(gimbalrotor_navigation
  ${ROS_DEPENDENCIES}
)
target_include_directories(gimbalrotor_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(
  aerial_robot_model
  plugins/robot_model_plugins.xml
)

pluginlib_export_plugin_description_file(
  aerial_robot_control
  plugins/control_plugins.xml
)

pluginlib_export_plugin_description_file(
  aerial_robot_navigation
  plugins/navigation_plugin.xml
)

install(
  TARGETS
    gimbalrotor_robot_model
    gimbalrotor_controller
    gimbalrotor_navigation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch plugins robots urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  gimbalrotor_robot_model
  gimbalrotor_controller
  gimbalrotor_navigation
)
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
