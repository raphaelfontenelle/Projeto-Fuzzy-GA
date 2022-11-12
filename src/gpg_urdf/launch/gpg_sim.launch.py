# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gpg_urdf"),
            "controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gpg_remote"), "gpg_remote.rviz"]
    )

    
    camera_config = os.path.join(
      get_package_share_directory('gpg_remote'),
      'camera.yaml'
      )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_controller", "--controller-manager", "/controller_manager"],
    )

    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic","robot_description", "-entity","rapharobot"],
    )
    
    #go = Node(
    #    package="gpg_urdf",
    #    executable="go",
    #    output='screen'
    #)
    
    #goto = Node(
    #    package="gpg_urdf",
    #    executable="reach_goal",
    #    output='screen'
    #)
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner, servo_controller_spawner],
        )
    )
    
    sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch/gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([FindPackageShare('gpg_urdf'), "world.sdf"])
            }.items()
        )
        
    sim_launch_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gpg_urdf'),
                    'display.launch.py'
                ])
            ]),
        )        
        
    nodes = [
        #control_node,
        sim_launch_ros,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        sim_launch,
        spawner,
        #go,
        #goto
    ]

    return LaunchDescription(nodes)
