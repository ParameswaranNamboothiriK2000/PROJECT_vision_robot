#!/bin/bash
# entrypoint.sh — Source ROS 2 environment on container start
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null || true

echo "🤖 ROS 2 Humble environment ready!"
echo "   Workspace: /root/ros2_ws"
echo "   Package:   vision_robot_controller"
exec "$@"
