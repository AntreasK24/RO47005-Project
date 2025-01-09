#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


def zigzag_trajectory(t,amplitude=1,period=5, is_xy = True):
    if is_xy: 
        x = amplitude * (t // period) * (-1) ** (t // period)
        y = t % period
        z = 1.0
        
    else: # XZ plane when is_xy = False
        x = amplitude * (t // period) * (-1) ** (t // period)
        y = 0.0
        z = t % period + 1 # with an offset of 1 m in z axis

    return x, y, z

class ZigZagTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('zigzag_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'trajectory/pub/zigzag', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        x, y, z = zigzag_trajectory(self.t) # in XY or XZ plane

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={z}')
        self.t += 0.1  

def main(args=None):
    rclpy.init(args=args)

    node = ZigZagTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
