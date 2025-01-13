#Imports go here

import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Bool
from geometry_msgs.msg import Twist,Pose
import drone_msgs.msg
from gym_pybullet_drones.envs import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

import pybullet as p



class DroneSimulator(Node):  
    #Constructor
    def __init__(self):
        super().__init__('drone_simulator')
        #Subscriber
        self.velocity_subscriber = self.create_subscription(Twist,'/cmd_vel',self.velocity_callback,10)
        self.avoid_positions_subscriber = self.create_subscription(Float64MultiArray,'/avoid_pos',self.avoid_positions_callback,10)
        #Publisher
        self.pose_publisher = self.create_publisher(Pose, '/pose', 10)
        self.obstacles_publisher = self.create_publisher(drone_msgs.msg.SphereArray,'/spheres', 2)
        #Timer
        timer_period = 1.0/240.0
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.num_points = 20       
        self.num_dimensions = 3    

        self.velocity_subscriber
        self.env = VelocityAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, ctrl_freq=240, gui=True)
        self.obs = self.env.reset()  

        self.add_obstacles()

    

    def add_obstacles(self):
        #self.create_obstacles(shape="cube",position=[1,1,0],scale=(0.5,0.5,0.5),color=(0,0,1,1))
        #self.create_obstacles(shape="cylinder",position=[1,0,0],scale=(0.1,1,2),color=(0,0.7,0.5,1))
        #self.create_obstacles(shape="sphere",position=[2,2,2],scale=(0.5,0.5,0.5),color=(0,0.7,0.5,1))

        new_position = np.random.uniform(low=1, high=5, size=(3,))
        self.create_obstacles(shape="sphere", position=new_position, scale=(0.5, 0.5, 0.5), color=(0,0.7,0.5,1))

        self.sphere_array_msg = drone_msgs.msg.SphereArray()
        sphere = drone_msgs.msg.Sphere()
        for _ in range(3):
            new_position = np.random.uniform(low=1, high=5, size=(3,))
            sphere = drone_msgs.msg.Sphere()
            sphere.position.x = new_position[0]
            sphere.position.y = new_position[1]
            sphere.position.z = new_position[2]
            sphere.radius = 0.5
            self.sphere_array_msg.spheres.append(sphere)

        self.obstacles_publisher.publish(self.sphere_array_msg)
        for sphere in self.sphere_array_msg.spheres:
            self.create_obstacles(shape="sphere", position=[sphere.position.x, sphere.position.y, sphere.position.z], scale=(sphere.radius, sphere.radius, sphere.radius), color=(0,0.7,0.5,1))


    def avoid_positions_callback(self,msg):
        data_list = msg.data
        positions_array = np.array(data_list).reshape((self.num_points, self.num_dimensions))

        for pos in positions_array:
            self.create_obstacles(shape="sphere",position=pos,scale=(0.5,0.5,0.5),color=(0,0.7,0.5,1))

    

    def create_obstacles(self,shape="cube",position=[0,0,0],color=[1,0,0,1],scale=(1,1,1)):
        if shape == 'cube':
            collision_shape = p.createCollisionShape(p.GEOM_BOX,halfExtents=scale)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=scale, rgbaColor=color) 
        elif shape == 'cylinder':
            collision_shape = p.createCollisionShape(p.GEOM_CYLINDER,radius=scale[0], height=scale[2])
            visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=scale[0], length=scale[2], rgbaColor=color)
        elif shape == 'sphere':
            collision_shape = p.createCollisionShape(p.GEOM_SPHERE,radius=scale[0])
            visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=scale[0], rgbaColor=color)


        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape, baseVisualShapeIndex=visual_shape, basePosition=position)

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
        self.obstacles_publisher.publish(self.sphere_array_msg)
        #Give velocity commands to drone and publish position
        pose_message = Pose()

        obs, reward, done, truncated, info = self.env.step(self.current_velocity)

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



