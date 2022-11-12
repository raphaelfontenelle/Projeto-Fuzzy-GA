from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

from ament_index_python.packages import get_package_share_path
import xacro

def generate_launch_description():
    # Get URDF
    #urdf = os.path.join(get_package_share_directory('gpg_urdf'), 'robot.urdf')
    #with open(urdf, 'r') as infp:
    #  robot_description_content = infp.read()

    urdf = get_package_share_path('gpg_urdf') / 'robot.urdf'
    robot_description_content = xacro.process_file(urdf).toxml()


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
