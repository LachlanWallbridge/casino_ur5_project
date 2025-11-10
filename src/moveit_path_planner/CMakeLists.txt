cmake_minimum_required(VERSION 3.10)
project(moveit_path_planner)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)

# --- Executable 1: main MoveIt planning server ---
add_executable(moveit_path_planning_server
  src/moveit_path_planning_server.cpp
)

ament_target_dependencies(moveit_path_planning_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  custom_interface
  ompl
)

# --- Executable 2: Cartesian demo ---
add_executable(cartesian_move_demo
  src/cartesian_move_demo.cpp
)



ament_target_dependencies(cartesian_move_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  custom_interface
  ompl
)

# --- Install both executables ---
install(TARGETS
  moveit_path_planning_server
  cartesian_move_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
