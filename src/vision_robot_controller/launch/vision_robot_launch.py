#!/usr/bin/env python3
"""
PART 4 v2 — vision_robot_launch.py (FIXED)
Fix: kinematics.yaml loaded as Python dict (not as --params-file)
     vision_node installed via scripts/ directory
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, "r") as f:
        return f.read()


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    # ── Robot description (URDF via xacro) ────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "config", "panda.urdf.xacro",
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── SRDF (semantic description) ────────────────────────
    robot_description_semantic_content = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # ── Kinematics — loaded as DICT, NOT as file path ──────
    # This is the fix: kinematics.yaml lacks ros__parameters header
    # so it cannot be passed as --params-file directly
    kinematics_raw = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )
    # Wrap under robot_description_kinematics namespace
    kinematics_yaml = {"robot_description_kinematics": kinematics_raw}

    # ── OMPL planning pipeline ─────────────────────────────
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    # ── Trajectory execution ───────────────────────────────
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["panda_arm_controller", "panda_hand_controller"]
        },
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── ros2_controllers.yaml path ─────────────────────────
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config", "ros2_controllers.yaml"
    )

    # ── NODE 1: robot_state_publisher ─────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── NODE 2: move_group ─────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,          # dict, not file path
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
        ],
    )

    # ── NODE 3: ros2_control_node ──────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        output="screen",
        parameters=[robot_description, ros2_controllers_path],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller",
                   "--controller-manager", "/controller_manager"],
    )
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller",
                   "--controller-manager", "/controller_manager"],
    )

    # ── NODE 4: RViz2 ──────────────────────────────────────
    rviz_config_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    # ── NODE 5: Our moveit_controller (delayed 6s) ─────────
    moveit_controller_node = Node(
        package="vision_robot_controller",
        executable="moveit_controller",
        name="moveit_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,          # dict, not file path
            {"use_sim_time": False},
        ],
    )

    delayed_moveit_controller = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="[6s] Starting moveit_controller node..."),
            moveit_controller_node,
        ]
    )

    return LaunchDescription([
        LogInfo(msg="=== Vision Robot — Master Launch (v2 fixed) ==="),
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
        move_group_node,
        rviz_node,
        delayed_moveit_controller,
        LogInfo(msg="Run vision node: DISPLAY=:0 QT_QPA_PLATFORM=xcb ros2 run vision_robot_controller vision_node.py"),
    ])
