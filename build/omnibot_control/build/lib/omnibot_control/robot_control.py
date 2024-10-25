#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from math import hypot


class RobotControlNode(Node):
    def __init__(self, robot_name, other_robots, var):
        super().__init__(f'{robot_name}_control')
        #super().__init__(node_name)

        self.position = None

        self.robot_name = robot_name
        self.other_robots = other_robots
        self.var = var

        # Create a TF buffer and listener to listen to transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Velocity publisher for this robot
        self.velocity_publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)

        
        # Timer to update the control law every 0.1 seconds
        self.create_timer(0.1, self.update_velocity)

        # Print or log the parameters to check if they are being passed correctly
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Other robots: {self.other_robots}')

    
    def get_robot_position(self, robot_name):
        try:
            # Lookup transformation from world frame to robot's frame
            trans = self.tf_buffer.lookup_transform(f'{robot_name}', 'empty', rclpy.time.Time())
            return (trans.transform.translation.x, trans.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"Could not transform {robot_name}: {str(e)}")
            return None
        

    def update_velocity(self):

        var = self.var
        # Get this robot's position
        position = self.get_robot_position(self.robot_name)
        if position is None:
            return  # This robot's position unknown yet

        avg_x, avg_y = 0.0, 0.0
        count = 0

        for robot in self.other_robots:
            other_position = self.get_robot_position(robot)
            if other_position is not None:
                avg_x += other_position[0]
                avg_y += other_position[1]
                count += 1

        if count > 0:
            avg_x /= count
            avg_y /= count

            # P control
            distance_x = avg_x - position[0]
            distance_y = avg_y - position[1]

            velocity_msg = Twist()
            #velocity_msg.linear.x = 0.05 * distance_x  
            #velocity_msg.linear.y = 0.05 * distance_y  

            velocity_msg.linear.x = var[0]
            velocity_msg.linear.y = var[1]
            velocity_msg.angular.z = 0.0

            self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('parameter_loader')
    robot_name = node.declare_parameter('robot_name', 'robot_1').get_parameter_value().string_value
    other_robots = node.declare_parameter('other_robots', ['robot_2']).get_parameter_value().string_array_value
    var = node.declare_parameter('var', [0.5, -0.5]).get_parameter_value().double_array_value
    node.destroy_node()


    control_node = RobotControlNode(robot_name, other_robots, var)
    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
