cmake_minimum_required(VERSION 3.8)
project(vision_robot_controller)

# ── C++17 — REQUIRED for std::clamp, std::optional, std::string_view ────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Generate compile_commands.json so VS Code IntelliSense sees C++17 ────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake                  REQUIRED)
find_package(rclcpp                       REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs                REQUIRED)
find_package(tf2_ros                      REQUIRED)
find_package(tf2_geometry_msgs            REQUIRED)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(moveit_controller
  src/moveit_controller.cpp)

ament_target_dependencies(moveit_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS moveit_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
