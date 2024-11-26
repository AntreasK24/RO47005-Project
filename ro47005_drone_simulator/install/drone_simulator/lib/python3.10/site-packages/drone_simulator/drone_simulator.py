#Imports go here

import numpy as np
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Pose
from gym_pybullet_drones.envs import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics


env = VelocityAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, ctrl_freq=240, gui=True)
obs = env.reset()  


class DroneSimulator(Node):  
    #Constructor
    def __init__(self):
        super().__init__('drone_simulator')
        #Subscriber
        self.velocity_subscriber = self.create_subscription(Twist,'/drone_velocity',self.velocity_callback,10)
        #Publisher
        self.pose_publisher = self.create_publisher(Pose, '/drone_pose', 10)
        #Timer
        timer_period = 1.0/240.0
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.velocity_subscriber

    #Variables for controlling drone 
    current_linear_velocity = np.array([[0.0, 0.0, 0.0, 1.0]])
    current_angular_velocity = np.array([[0.0, 0.0, 0.0]])
    current_velocity = np.hstack((current_linear_velocity,current_angular_velocity))


    def velocity_callback(self,msg):
        #Get linear and angular velocity and stack them into a single vector
        self.current_linear_velocity = np.array([[msg.linear.x,msg.linear.y,msg.linear.z,1]])
        self.current_angular_velocity = np.array([[msg.angular.x,msg.angular.y,msg.angular.z]])
        self.current_velocity = np.hstack((self.current_linear_velocity,self.current_angular_velocity))
        

    def timer_callback(self):
        #Give velocity commands to drone and publish position
        pose_message = Pose()

        obs, reward, done, truncated, info = env.step(self.current_velocity)

        position = obs[0][0:3]
        pose_message.position.x = position[0]
        pose_message.position.y = position[1]
        pose_message.position.z = position[2]

        quaternion = obs[0][3:7]
        pose_message.orientation.x = quaternion[0]
        pose_message.orientation.y = quaternion[1]
        pose_message.orientation.z = quaternion[2]
        pose_message.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_message)

    

def main(args=None):
    rclpy.init(args=args)
    drone_simulator = DroneSimulator()
    rclpy.spin(drone_simulator)
    drone_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



#env.close()