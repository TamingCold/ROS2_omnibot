from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():

    var1 = np.random.normal(-1.0, 1.0, 2).tolist()
    var2 = np.random.normal(-1.0, 1.0, 2).tolist()

    ld = LaunchDescription()

    robot_control_1 = Node(
        package='omnibot_control',  
        executable='robot_control',  
        name='robot_1_control',
        parameters=[{ 
            'robot_name': 'robot_1',
            'other_robots': ['robot_2'],
            'var': var1
        }],
        output = 'screen'
    )

    robot_control_2 = Node(
        package='omnibot_control',  
        executable='robot_control', 
        name='robot_2_control',
        parameters=[{ 
            'robot_name': 'robot_2',
            'other_robots': ['robot_1'],
            'var': var2
        }],
        output = 'screen'
    )

    ld.add_action(robot_control_1)
    ld.add_action(robot_control_2)

    
    return ld