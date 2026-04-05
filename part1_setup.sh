#!/bin/bash
# ============================================================
#  PART 1 — ROS 2 Vision-to-Motion Robot Project
#  Full Setup: Dependencies + Workspace Scaffold
#  System: Ubuntu 22.04 | ROS 2 Humble | GTX 1650 | CUDA 12.3
# ============================================================

set -e  # Stop on any error

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PART 1: Installing Dependencies               ║"
echo "╚══════════════════════════════════════════════════╝"

# --- Step 1: Source ROS 2 ---
source /opt/ros/humble/setup.bash
echo "✅ ROS 2 Humble sourced"

# --- Step 2: Update apt ---
sudo apt update

# --- Step 3: Install MoveIt 2 ---
echo ""
echo "📦 Installing MoveIt 2..."
sudo apt install -y ros-humble-moveit
echo "✅ MoveIt 2 installed"

# --- Step 4: Install Panda robot MoveIt config ---
echo ""
echo "📦 Installing Panda MoveIt config..."
sudo apt install -y ros-humble-moveit-resources-panda-moveit-config
sudo apt install -y ros-humble-moveit-resources-panda-description
echo "✅ Panda robot config installed"

# --- Step 5: Install Gazebo ROS integration ---
echo ""
echo "📦 Installing Gazebo ROS packages..."
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-gazebo-ros2-control
echo "✅ Gazebo ROS bridge installed"

# --- Step 6: Install ROS 2 control ---
echo ""
echo "📦 Installing ROS 2 Control..."
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-controller-manager
echo "✅ ROS 2 Control installed"

# --- Step 7: Install Python vision dependencies ---
echo ""
echo "📦 Installing Python packages (YOLO11, OpenCV)..."
pip3 install ultralytics
pip3 install opencv-python
pip3 install numpy --upgrade
echo "✅ Python vision packages installed"

# --- Step 8: Install extra ROS tools ---
echo ""
echo "📦 Installing ROS 2 dev tools..."
sudo apt install -y python3-rosdep python3-vcstool
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-rviz2
echo "✅ ROS 2 tools installed"

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PART 1: Creating Workspace Scaffold           ║"
echo "╚══════════════════════════════════════════════════╝"

# --- Step 9: Create workspace ---
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

echo "✅ Workspace created at ~/ros2_ws"

# --- Step 10: Create the ROS 2 package ---
ros2 pkg create --build-type ament_cmake vision_robot_controller \
  --dependencies rclcpp rclcpp_action geometry_msgs std_msgs sensor_msgs \
               moveit_ros_planning_interface moveit_core

echo "✅ ROS 2 package created: vision_robot_controller"

# --- Step 11: Create Python vision node directory ---
mkdir -p ~/ros2_ws/src/vision_robot_controller/scripts
mkdir -p ~/ros2_ws/src/vision_robot_controller/launch
mkdir -p ~/ros2_ws/src/vision_robot_controller/config
mkdir -p ~/ros2_ws/src/vision_robot_controller/rviz

echo "✅ Folder structure created"

# --- Step 12: Create placeholder vision_node.py ---
cat > ~/ros2_ws/src/vision_robot_controller/scripts/vision_node.py << 'PYEOF'
#!/usr/bin/env python3
"""
vision_node.py — PART 2 will fill this file completely.
Placeholder to confirm package structure is correct.
"""
import rclpy
from rclpy.node import Node

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision Node placeholder — Part 2 coming next!')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF
chmod +x ~/ros2_ws/src/vision_robot_controller/scripts/vision_node.py
echo "✅ vision_node.py placeholder created"

# --- Step 13: Create placeholder moveit_controller.cpp ---
cat > ~/ros2_ws/src/vision_robot_controller/src/moveit_controller.cpp << 'CPPEOF'
// moveit_controller.cpp — PART 3 will fill this file completely.
// Placeholder to confirm package structure is correct.
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_controller");
  RCLCPP_INFO(node->get_logger(), "MoveIt Controller placeholder — Part 3 coming next!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
CPPEOF
echo "✅ moveit_controller.cpp placeholder created"

# --- Step 14: Write package.xml ---
cat > ~/ros2_ws/src/vision_robot_controller/package.xml << 'XMLEOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_robot_controller</name>
  <version>1.0.0</version>
  <description>
    Autonomous Vision-to-Motion Robotic Inspection Pipeline.
    Python YOLO11 vision node + C++ MoveIt 2 controller for Panda 7-DOF arm.
  </description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>moveit_ros_planning_interface</depend>
  <depend>moveit_core</depend>
  <depend>moveit_visual_tools</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
XMLEOF
echo "✅ package.xml written"

# --- Step 15: Write CMakeLists.txt ---
cat > ~/ros2_ws/src/vision_robot_controller/CMakeLists.txt << 'CMAKEEOF'
cmake_minimum_required(VERSION 3.8)
project(vision_robot_controller)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# -----------------------------------------------
# C++ Executable: moveit_controller
# (Part 3 will add full source here)
# -----------------------------------------------
add_executable(moveit_controller src/moveit_controller.cpp)
ament_target_dependencies(moveit_controller
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS moveit_controller
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------
# Python Scripts: vision_node.py
# -----------------------------------------------
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------
# Launch files and config
# -----------------------------------------------
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
CMAKEEOF
echo "✅ CMakeLists.txt written"

# --- Step 16: Create placeholder launch file ---
cat > ~/ros2_ws/src/vision_robot_controller/launch/demo_simulation.launch.py << 'LAUNCHEOF'
# demo_simulation.launch.py
# PART 5 will fill this with full Gazebo + MoveIt 2 + Panda launch.
# Placeholder to confirm launch folder structure.
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([])
LAUNCHEOF
echo "✅ Launch file placeholder created"

# --- Step 17: Create __init__.py for Python package ---
mkdir -p ~/ros2_ws/src/vision_robot_controller/vision_robot_controller
touch ~/ros2_ws/src/vision_robot_controller/vision_robot_controller/__init__.py
echo "✅ Python package __init__.py created"

# --- Step 18: Build the workspace ---
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PART 1: Building Workspace (First Build)      ║"
echo "╚══════════════════════════════════════════════════╝"
cd ~/ros2_ws
colcon build --symlink-install
echo "✅ First build complete"

# --- Step 19: Source the workspace ---
source ~/ros2_ws/install/setup.bash
echo "✅ Workspace sourced"

# --- Step 20: Add auto-source to .bashrc ---
echo "" >> ~/.bashrc
echo "# ROS 2 Vision Robot Project" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export CUDA_HOME=/usr/local/cuda" >> ~/.bashrc
echo "export PATH=\$PATH:\$CUDA_HOME/bin" >> ~/.bashrc
echo "✅ Added ROS 2 + CUDA to ~/.bashrc (auto-sources on every terminal open)"

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PART 1: Verification Tests                    ║"
echo "╚══════════════════════════════════════════════════╝"

echo ""
echo "--- Test 1: ROS 2 package list ---"
ros2 pkg list | grep -E "moveit|panda|vision_robot" || echo "checking..."

echo ""
echo "--- Test 2: Panda URDF available ---"
ros2 pkg prefix moveit_resources_panda_description 2>/dev/null && echo "✅ Panda URDF found" || echo "❌ Panda URDF missing"

echo ""
echo "--- Test 3: YOLO11 import test ---"
python3 -c "from ultralytics import YOLO; print('✅ YOLO11 import OK')" 2>/dev/null || echo "❌ YOLO11 not importable"

echo ""
echo "--- Test 4: Our package exists ---"
ros2 pkg list | grep vision_robot_controller && echo "✅ Our package found in ROS 2" || echo "❌ Package not found — rebuild needed"

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PART 1 COMPLETE ✅                            ║"
echo "╠══════════════════════════════════════════════════╣"
echo "║  Workspace  : ~/ros2_ws                         ║"
echo "║  Package    : vision_robot_controller           ║"
echo "║  Next step  : Reply PART 2 → vision_node.py    ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
echo "IMPORTANT: Run this in a NEW terminal after done:"
echo "  source ~/.bashrc"
