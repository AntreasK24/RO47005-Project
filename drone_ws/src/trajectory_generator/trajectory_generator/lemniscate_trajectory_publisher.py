#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


def lemniscate_trajectory(t, a=1, is_xy = True):
    # reference: https://proofwiki.org/wiki/Definition:Lemniscate_of_Bernoulli
    if is_xy:
        x = (a*math.sqrt(2)* math.cos(t)) / (1 + math.sin(t)**2) # focal point a 
        y = (a*math.sqrt(2)* math.sin(t) * math.cos(t) )/ (1 + math.sin(t)**2)
        z = 1.0 # constant altitude of 1 m
    else:
        x = (a*math.sqrt(2)* math.cos(t)) / (1 + math.sin(t)**2) # focal point a 
        y = 0.0
        z = (a*math.sqrt(2)* math.sin(t) * math.cos(t) )/ (1 + math.sin(t)**2) + 1  # with an offset of 1 m in z axis    

    return x, y, z

class LemniscateTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('lemniscate_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'trajectory/pub/lemniscate', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.t = 0.0

    def publish_trajectory(self):
        x, y, z = lemniscate_trajectory(self.t)

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={z}')
        self.t += 0.1  

def main(args=None):
    rclpy.init(args=args)

    node = LemniscateTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
