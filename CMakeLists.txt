cmake_minimum_required(VERSION 3.8)
project(rb10_rmpflow_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CASADI_ROOT "/home/song/ros2_ws/third_party/casadi/install")
set(CASADI_INCLUDE_DIR "${CASADI_ROOT}/include")
set(CASADI_LIBRARY "${CASADI_ROOT}/lib/libcasadi.so")
set(PINOCCHIO_ROOT "/opt/ros/humble")
find_path(
  PINOCCHIO_INCLUDE_DIR
  pinocchio/multibody/model.hpp
  HINTS "${PINOCCHIO_ROOT}/include"
  REQUIRED
)
find_library(
  PINOCCHIO_DEFAULT_LIBRARY
  pinocchio_default
  HINTS "${PINOCCHIO_ROOT}/lib/x86_64-linux-gnu"
  REQUIRED
)
find_library(
  PINOCCHIO_PARSERS_LIBRARY
  pinocchio_parsers
  HINTS "${PINOCCHIO_ROOT}/lib/x86_64-linux-gnu"
  REQUIRED
)

add_library(rb10_casadi_graph
  src/casadi_task_graph.cpp
)
target_include_directories(rb10_casadi_graph PRIVATE include)
target_include_directories(rb10_casadi_graph SYSTEM PRIVATE ${CASADI_INCLUDE_DIR})
set_target_properties(
  rb10_casadi_graph PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(rb10_casadi_graph Eigen3::Eigen ${CASADI_LIBRARY})

add_library(rb10_pinocchio_rmp
  src/pinocchio_model.cpp
  src/pinocchio_direct_solver.cpp
)
target_include_directories(rb10_pinocchio_rmp PUBLIC include)
target_include_directories(
  rb10_pinocchio_rmp SYSTEM PUBLIC
  ${PINOCCHIO_INCLUDE_DIR}
)
target_include_directories(
  rb10_pinocchio_rmp SYSTEM PRIVATE
  ${CASADI_INCLUDE_DIR}
)
set_target_properties(
  rb10_pinocchio_rmp PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(
  rb10_pinocchio_rmp
  Eigen3::Eigen
  Boost::filesystem
  Boost::serialization
  ${PINOCCHIO_DEFAULT_LIBRARY}
  ${PINOCCHIO_PARSERS_LIBRARY}
  rb10_casadi_graph
  ${CASADI_LIBRARY}
)

add_library(rb10_learning_support
  src/rmp_learning_interface.cpp
  src/rmp_learning_stack.cpp
)
target_include_directories(rb10_learning_support PUBLIC include)
target_link_libraries(
  rb10_learning_support
  Eigen3::Eigen
  rb10_pinocchio_rmp
)

add_executable(rmpflow_controller src/rmpflow_controller_node.cpp)
target_include_directories(rmpflow_controller PRIVATE include)
set_target_properties(
  rmpflow_controller PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(
  rmpflow_controller
  Eigen3::Eigen
  rb10_pinocchio_rmp
  ${CASADI_LIBRARY}
)
ament_target_dependencies(
  rmpflow_controller
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_executable(interactive_goal src/interactive_goal_marker.cpp)
target_include_directories(interactive_goal PRIVATE include)
set_target_properties(
  interactive_goal PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(interactive_goal Eigen3::Eigen)
ament_target_dependencies(
  interactive_goal
  geometry_msgs
  interactive_markers
  rclcpp
  std_msgs
  visualization_msgs
)

add_executable(obstacle_manager src/obstacle_manager.cpp)
target_include_directories(obstacle_manager PRIVATE include)
set_target_properties(
  obstacle_manager PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(obstacle_manager Eigen3::Eigen)
ament_target_dependencies(
  obstacle_manager
  geometry_msgs
  interactive_markers
  rclcpp
  std_msgs
  visualization_msgs
)

add_executable(tof_ray_visualizer src/tof_ray_visualizer.cpp)
target_include_directories(tof_ray_visualizer PRIVATE include)
set_target_properties(
  tof_ray_visualizer PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(tof_ray_visualizer Eigen3::Eigen)
ament_target_dependencies(
  tof_ray_visualizer
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

add_executable(proximity_obstacle_bridge src/proximity_obstacle_bridge.cpp)
target_include_directories(proximity_obstacle_bridge PRIVATE include)
set_target_properties(
  proximity_obstacle_bridge PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib"
)
target_link_libraries(proximity_obstacle_bridge Eigen3::Eigen)
ament_target_dependencies(
  proximity_obstacle_bridge
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

install(
  TARGETS
    rb10_pinocchio_rmp
    rb10_casadi_graph
    rb10_learning_support
    rmpflow_controller
    interactive_goal
    obstacle_manager
    tof_ray_visualizer
    proximity_obstacle_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rmp_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
