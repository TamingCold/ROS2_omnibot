# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    N = 2

    ld = LaunchDescription()

    for i in range(1,N + 1):
        robot_name = f'robot_{i}'
        other_robots = [f'robot_{j}' for j in range(1, N + 1) if j != i]
        x_pose = LaunchConfiguration(f'x_pose_{i}', default=str(i * 1.5))
        y_pose = LaunchConfiguration(f'y_pose_{i}', default='0.0')

        declare_x_position_cmd = DeclareLaunchArgument(
            f'x_pose_{i}', default_value=str(i * 1.5),
            description=f'Specify x position of {robot_name}')

        declare_y_position_cmd = DeclareLaunchArgument(
            f'y_pose_{i}', default_value='0.0',
            description=f'Specify y position of {robot_name}')
        
        urdf_path = os.path.join(
        get_package_share_directory('omnibot_ignition'),
        'urdf',
        'omni_bot.sdf'
        )

        start_gazebo_ros_spawner_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '1.01'
            ],
            output='screen',
        )

        

        ld.add_action(declare_x_position_cmd)
        ld.add_action(declare_y_position_cmd)
        ld.add_action(start_gazebo_ros_spawner_cmd)
    


    bridge_params = os.path.join(
        get_package_share_directory('omnibot_ignition'),
        'params',
        'omni_control_config.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot_1',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    omni_drive_cmd = Node(package='omnibot_ignition',
        executable='omni_drive',
        name='omni_drive',
        namespace='robot_1'
    )

    
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(omni_drive_cmd)

    


    return ld

'''
    robot_name = 'robot_1'
    other_robots = ['robot_2']

    control_node = Node(
        package='omnibot_control',
        executable='robot_control',
        name=f'{robot_name}_control',
        parameters=[{
            'robot_name': robot_name,
            'other_robots': other_robots
        }],
        output = 'screen'
    )

    ld.add_action(control_node)
'''


