# 🤖 Autonomous Vision-to-Motion Robotic Inspection Pipeline

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-orange)](https://moveit.ros.org/)
[![YOLO11](https://img.shields.io/badge/YOLO-11-green)](https://ultralytics.com/)
[![CUDA](https://img.shields.io/badge/CUDA-Enabled-76b900)](https://developer.nvidia.com/cuda-toolkit)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](https://isocpp.org/)

> A fully autonomous closed-loop robotic system that uses deep learning to detect real-world objects and commands a 7-DOF Panda robot arm to track and follow them in real time — without any human intervention.

---

## 📸 Demo

**Live OpenCV Window — YOLO11 Detection + 3D Coordinates**

```
╔══════════════════════════════════════════╗
║  ROS2 Vision Node | Detections: 883     ║
║  bottle  0.86                           ║
║  3D: (0.53, 0.25, 0.30) m              ║
╚══════════════════════════════════════════╝
```

- 🟢 Green bounding box drawn around detected bottle
- 🔴 Red crosshair on object centroid
- 📐 Live 3D workspace coordinates (meters) displayed on frame
- 🤖 Panda arm in RViz physically moves to track the object

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  ROS 2 NETWORK (DDS / Topic Bus)            │
│                                                             │
│  ┌──────────────────┐        ┌──────────────────────────┐  │
│  │  Python Vision   │        │   C++ MoveIt Controller  │  │
│  │  Node            │        │   Node                   │  │
│  │                  │        │                          │  │
│  │  • Webcam feed   │/target_│  • MoveGroupInterface    │  │
│  │  • YOLO11 CUDA   │  pose  │  • OMPL IK Solver        │  │
│  │  • 2D to 3D math │───────►│  • MultiThreadedExecutor │  │
│  │  • Moving avg    │(Point) │  • Safety bounds check   │  │
│  │    filter        │        │  • Atomic cooldown timer │  │
│  │  • Pixel to m    │        │  • Quaternion orientation│  │
│  └──────────────────┘        └──────────────────────────┘  │
│                                           │                 │
│                                           ▼                 │
│                              ┌──────────────────────────┐  │
│                              │   MoveIt 2 + RViz        │  │
│                              │   Panda 7-DOF Arm        │  │
│                              └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Key Features

- Closed-loop autonomous control — no pre-programmed positions
- Real-time YOLO11 detection running on CUDA GPU (~30 FPS)
- 2D pixel to 3D workspace projection with moving average smoothing
- Collision-free motion planning via OMPL/RRTConnect in ~23ms
- Atomic execution cooldown prevents command flooding
- Quaternion-based downward end-effector orientation
- Table collision object in planning scene for safety
- Live OpenCV debug overlay with bounding box, centroid, 3D coords

---

## 🛠️ Tech Stack

| Layer | Technology |
|-------|-----------|
| OS | Ubuntu 22.04 LTS |
| Robot Framework | ROS 2 Humble Hawksbill |
| Motion Planning | MoveIt 2 + OMPL (RRTConnect) |
| Robot Model | Franka Panda 7-DOF |
| Deep Learning | YOLO11n (Ultralytics) |
| Compute | NVIDIA GPU (CUDA) |
| Vision Library | OpenCV 4.5 |
| Language (Vision) | Python 3.10 |
| Language (Control) | C++17 |
| Build System | Colcon + CMake (ament_cmake) |

---

## 📁 Project Structure

```
PROJECT_vision_robot/
├── src/
│   └── vision_robot_controller/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── scripts/
│       │   └── vision_node.py          # Python YOLO11 vision node
│       ├── src/
│       │   └── moveit_controller.cpp   # C++ MoveIt controller node
│       ├── launch/
│       │   ├── vision_robot_launch.py  # Full system launch
│       │   └── run_controller.launch.py
│       └── config/
│           └── rviz_config.rviz
├── build/
├── install/
└── log/
```

---

## ⚙️ Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# MoveIt 2
sudo apt install ros-humble-moveit

# Panda MoveIt Config
sudo apt install ros-humble-moveit-resources-panda-moveit-config
sudo apt install ros-humble-moveit-resources-panda-description

# ROS 2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Python dependencies
pip install ultralytics opencv-python numpy
```

---

## 🔧 Build and Install

```bash
cd ~/PROJECT_vision_robot
colcon build --packages-select vision_robot_controller --symlink-install
source install/setup.bash
```

---

## ▶️ Running the System

### Terminal 1 — MoveIt Backend (start first)
```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
Wait for: `You can start planning now!`

### Terminal 2 — MoveIt Controller
```bash
source /opt/ros/humble/setup.bash
source ~/PROJECT_vision_robot/install/setup.bash
ros2 launch vision_robot_controller run_controller.launch.py
```
Wait for: `System FULLY OPERATIONAL`

### Terminal 3 — Vision Node
```bash
source /opt/ros/humble/setup.bash
source ~/PROJECT_vision_robot/install/setup.bash
DISPLAY=:0 QT_QPA_PLATFORM=xcb ros2 run vision_robot_controller vision_node.py
```

---

## 📡 ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| /target_pose | geometry_msgs/PointStamped | 3D target from vision node |
| /joint_states | sensor_msgs/JointState | Robot joint feedback |
| /planning_scene | moveit_msgs/PlanningScene | Collision scene |

---

## 🧠 Algorithm Details

### Vision Node (Python)
1. Capture frame from webcam (640x480 at 30 FPS)
2. Run YOLO11n inference on GPU — detect bottle class
3. Select highest-confidence detection
4. Compute pixel centroid (cx, cy) of bounding box
5. Apply 2D to 3D back-projection: X = (cx - W/2) / W * scale
6. Apply moving average filter (window=5) for temporal smoothing
7. Publish geometry_msgs/PointStamped to /target_pose
8. Draw debug overlay: bounding box, centroid, 3D coords, frame count

### Controller Node (C++)
1. Subscribe to /target_pose with MultiThreadedExecutor
2. Check atomic cooldown timer (2.0s) to prevent flooding
3. Validate target within safety workspace bounds
4. Set quaternion downward orientation for end-effector
5. Call MoveGroupInterface setPoseTarget
6. Plan with OMPL RRTConnect (up to 3 retry attempts)
7. Execute trajectory if plan found
8. Update last-position tracker for distance filtering

---

## 📊 Performance Results

| Metric | Value |
|--------|-------|
| YOLO11 inference speed | ~30 FPS (CUDA) |
| Detection confidence | 0.55 to 0.87 |
| Planning time | ~23 to 76ms |
| Trajectory duration | 1.5 to 5.7 seconds |
| Total successful moves | 10+ per session |
| End-to-end latency | under 3 seconds |

---

## 🔮 Future Improvements

- [ ] Real depth estimation using stereo or depth camera
- [ ] Kalman filter for smoother trajectory prediction
- [ ] Multi-object tracking and target selection UI
- [ ] Gazebo simulation with full physics
- [ ] Camera intrinsic calibration for accurate 3D projection
- [ ] Grasp planning and pick-and-place extension

---

## 👨‍💻 Author

R&D Engineer — Computer Vision and Robotics
Specialization: 3D vision systems, depth cameras, pose estimation, industrial metrology
Tools: Python, C++, ROS 2, MoveIt 2, OpenCV, YOLO, Open3D, CUDA

---

## 📄 License

MIT License — free to use, modify, and distribute.

---

*Built with ROS 2 Humble · MoveIt 2 · YOLO11 · OpenCV · Ubuntu 22.04*
