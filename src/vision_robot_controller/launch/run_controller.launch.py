import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    panda_config = get_package_share_directory('moveit_resources_panda_moveit_config')

    urdf_xacro = os.path.join(panda_config, 'config', 'panda.urdf.xacro')
    srdf_path   = os.path.join(panda_config, 'config', 'panda.srdf')
    kin_path    = os.path.join(panda_config, 'config', 'kinematics.yaml')

    with open(srdf_path, 'r') as f:
        srdf = f.read()
    with open(kin_path, 'r') as f:
        kinematics = yaml.safe_load(f)

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str)

    return LaunchDescription([
        Node(
            package='vision_robot_controller',
            executable='moveit_controller',
            name='moveit_controller',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': srdf,
                **kinematics,
            }]
        )
    ])
