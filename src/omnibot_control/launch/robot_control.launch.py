from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    robot_control_node = Node(
        package='omnibot_control',  # Name of the package containing the 'robot_control.py' script
        executable='robot_control',  # This matches the entry point defined in setup.py
        name='robot_1_control',
        output = 'screen'
    )

    # Return a LaunchDescription that includes the node
    return LaunchDescription([
        robot_control_node
    ])