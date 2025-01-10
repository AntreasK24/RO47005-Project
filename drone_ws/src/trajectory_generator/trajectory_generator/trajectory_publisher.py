#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
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


def lemniscate_trajectory(t, a=5, is_xy = True):
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


def linear_trajectory(t,c=0.05, is_x = True, is_y = False):
    x, y = 0.0, 0.0
    if is_x:
        x = c*t # constant c
    if is_y:
        y = c*t
    return x, y


def spiral_trajectory(t,r=1, c =0.05):
    x = (r + c*t) * math.cos(t)
    y = (r + c*t) * math.sin(t)
    z = c*t  # altitude increases linearly with constant co-eff c
    return x, y, z


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


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/target_pos', 10)

        self.subscriber_ = self.create_subscription(Bool, '/point_reached', self.reached_point_callback, 10)
        self.t = 0.0

        self.declare_parameter('trajectory_type', "Lemniscate")

        self.trajectory_type = self.get_parameter('trajectory_type').value



    def reached_point_callback(self, msg):
        if msg.data:
            
            if self.trajectory_type == "Circle": 
                x, y, z = circle_trajectory(self.t)
                self.t += 0.1
            elif self.trajectory_type == "Lemniscate":
                x, y, z = lemniscate_trajectory(self.t)
                self.t += 0.1
            elif self.trajectory_type == "Linear":
                x, y = linear_trajectory(self.t)
                z = 1.0
                self.t += 0.1
            elif self.trajectory_type == "Spiral":
                x, y, z = spiral_trajectory(self.t)
                self.t += 0.1
            elif self.trajectory_type == "Zigzag":
                x, y, z = zigzag_trajectory(self.t)
                self.t += 0.1

            point = Float64MultiArray()
            point.data = [x, y, z]

            self.publisher_.publish(point)
            self.get_logger().info(f'Publishing Point: x={x}, y={y}, z={z}')
        else:
            self.get_logger().info('Point not reached yet')

def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
