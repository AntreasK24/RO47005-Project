#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def linear_trajectory(t,c=0.05, is_x = True, is_y = False):
    x, y = 0.0, 0.0
    if is_x:
        x = c*t # constant c
    if is_y:
        y = c*t
    return x, y

class LinearTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('linear_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'trajectory/pub/linear', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        x, y = linear_trajectory(self.t) # can be in x and/or y

        point = Point()
        point.x = x
        point.y = y
        point.z = 1.0 # hovering at an attitude of 1 m

        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={point.z}')
        self.t += 0.1  

def main(args=None):
    rclpy.init(args=args)

    node = LinearTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
