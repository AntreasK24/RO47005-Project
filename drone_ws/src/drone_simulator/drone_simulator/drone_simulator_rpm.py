#Imports go here

import numpy as np
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import Float64MultiArray
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

import pybullet as p

class DroneSimulator(Node):
    #Constructor
    def __init__(self):
        super().__init__('drone_simulator')
        #Subscriber
        self.rpm_subsriber = self.create_subscription(Float64MultiArray,'/motor_rpm',self.rpm_callback,10)
        #Publisher
        self.pose_publisher = self.create_publisher(Pose, '/pose',10)
        #Timer
        timer_period = 1.0/240.0
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, gui=True)
        self.obs = self.env.reset() 

        self.add_obstacles()

    def add_obstacles(self):
        self.create_obstacles(shape="cube",position=[1,1,0],scale=(0.5,0.5,0.5),color=(0,0,1,1))
        self.create_obstacles(shape="cylinder",position=[1,0,0],scale=(0.1,1,2),color=(0,0.7,0.5,1))


    def create_obstacles(self,shape="cube",position=[0,0,0],color=[1,0,0,1],scale=(1,1,1)):
        if shape == 'cube':
            collision_shape = p.createCollisionShape(p.GEOM_BOX,halfExtents=scale)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=scale, rgbaColor=color) 
        elif shape == 'cylinder':
            collision_shape = p.createCollisionShape(p.GEOM_CYLINDER,radius=scale[0], height=scale[2])
            visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=scale[0], length=scale[2], rgbaColor=color)


        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=position)

    motor_rpm = np.zeros((1, 4))

    def rpm_callback(self,msg):
        #Get rpm and put it into a single vector
        print(msg.data)
        motor_rpm_data = np.array(msg.data).reshape(1, 4)  # Ensure motor RPM is reshaped correctly
        self.motor_rpm = motor_rpm_data
    def timer_callback(self):
        #Give velocity commands to drone and publish position
        pose_message = Pose()

        obs, reward, done, truncated, info = self.env.step(self.motor_rpm)

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


