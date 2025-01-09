#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


def spiral_trajectory(t,r=1, c =0.05):
    x = (r + c*t) * math.cos(t)
    y = (r + c*t) * math.sin(t)
    z = c*t  # altitude increases linearly with constant co-eff c
    return x, y, z

class SpiralTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('spiral_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'trajectory/pub/spiral', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        x, y, z = spiral_trajectory(self.t) # in xy or xz plane

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={z}')
        self.t += 0.1  

def main(args=None):
    rclpy.init(args=args)

    node = SpiralTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
