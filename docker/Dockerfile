# ============================================================
# Dockerfile — Autonomous Vision-to-Motion Robotic Inspection
# ROS 2 Humble + MoveIt 2 + YOLO11 + CUDA
# ============================================================

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

LABEL maintainer="R&D Engineer — Computer Vision and Robotics"
LABEL description="Autonomous Vision-to-Motion Robotic Inspection Pipeline"
LABEL version="1.0"

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# ── System dependencies ────────────────────────────────────
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
        git curl wget build-essential \
        python3-pip python3-colcon-common-extensions \
        python3-rosdep python3-vcstool \
        ros-humble-moveit \
        ros-humble-moveit-resources-panda-moveit-config \
        ros-humble-moveit-resources-panda-description \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-rviz2 \
        libgl1-mesa-glx libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies (YOLO11, OpenCV, Open3D) ──────────
RUN pip3 install --no-cache-dir \
    ultralytics \
    opencv-python-headless \
    numpy \
    open3d

# ── Set up ROS 2 workspace ─────────────────────────────────
ENV ROS_WS=/root/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# ── Copy project source ────────────────────────────────────
COPY src/ ${ROS_WS}/src/

# ── Install ROS dependencies & build ──────────────────────
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install && \
    echo "✅ Build complete"

# ── Source workspace in every new shell ───────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

# ── Entrypoint ─────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${ROS_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
