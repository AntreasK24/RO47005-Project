#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


def circle_trajectory(t,r=1, is_xy = True):
    if is_xy: 
        x = r*math.cos(t) # radius r
        y = r*math.sin(t)
        z = 1.0 # constant altitude of 1 m
        
    else: # when is_xy = False
        x = r*math.cos(t) # radius r
        y = 0.0
        z = r*math.sin(t) + 1 # with an offset of 1 m in z axis

    return x, y, z

class CircleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circle_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'trajectory/pub/circle', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        x, y, z = circle_trajectory(self.t) # in xy or xz plane

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={z}')
        self.t += 0.1  

def main(args=None):
    rclpy.init(args=args)

    node = CircleTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
