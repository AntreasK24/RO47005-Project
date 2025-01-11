import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point,PoseArray
from std_msgs.msg import Float64MultiArray,Bool
import drone_msgs.msg

import numpy as np
import transforms3d
import matplotlib.pyplot as plt

from drone_mpc_python.mpcDroneSolver import DroneMPCSolver
from mpl_toolkits.mplot3d import Axes3D
import timeit

class DroneMPCNode(Node):
    #Constructor
    def __init__(self):
        super().__init__('drone_mpc')

        self.hover = False
        self.positions = []

        self.Q = np.diag([10.0,10.0,10.0,4.0,4.0,4.0])
        self.R = np.diag([0.05,0.05,0.05])

        self.Q =  self.Q.flatten().tolist()
        self.R =  self.R.flatten().tolist()

        #Create MPC solver object
        self.drone_solver = DroneMPCSolver()

        self.declare_parameter('initial_state', [0.0,0.0,0.1125,0.0,0.0,0.0])
        self.declare_parameter('target_pos', [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

        self.declare_parameter('accel_max', 500)
        self.declare_parameter('N_horizon', 50)
        self.declare_parameter('prediction_period', 0.05)
        self.declare_parameter('Q',self.Q)
        self.declare_parameter('R', self.R)
        self.declare_parameter('drone_radius', 0.5)

        self.declare_parameter('d_min', 0.5)

        self.declare_parameter('noise', False)
        self.declare_parameter('noise_std_pos', 0.1)
        self.declare_parameter('noise_std_vel', 0.1)


        

        self.noise = self.get_parameter('noise').value
        self.noise_std_pos = self.get_parameter('noise_std_pos').value
        self.noise_std_vel = self.get_parameter('noise_std_vel').value

        self.Q = self.get_parameter('Q').value
        self.Q = np.array(self.Q).reshape(6,6)
        self.R = self.get_parameter('R').value
        self.R = np.array(self.R).reshape(3,3)
        self.accel_max = self.get_parameter('accel_max').value
        self.N_horizon = self.get_parameter('N_horizon').value
        self.prediction_period = self.get_parameter('prediction_period').value
        self.drone_radius = self.get_parameter('drone_radius').value

        self.new_pos = True

        num_points = 20        # Number of random points
        num_dimensions = 3     # Each point will have 3 dimensions (x, y, z)
        lower_bound = 1       # Lower bound of the range
        upper_bound = 6       # Upper bound of the range

        self.avoid_pos = np.random.uniform(low=lower_bound, high=upper_bound, size=(num_points, num_dimensions))


        #Set initial state and default target position (this  could be a ROS param)
        self.initial_state = np.array(self.get_parameter('initial_state').value)
        self.target_pos = np.array(self.get_parameter('target_pos').value)


        self.avoid_pos = None

        #Setup solver
        self.drone_solver.setup_solver(init_pos=self.initial_state,target_pos=self.target_pos,avoid_pos=self.avoid_pos,d_min=0.5)

        #Publisher
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reached_point_pub = self.create_publisher(Bool, '/point_reached',10)
        self.waypoint_pub = self.create_publisher(PoseArray,'/waypoints',1)

        #Subscribers
        self.current_pose_sub = self.create_subscription(Pose,'/pose',self.current_pose_callback,10)
        self.target_position_sub = self.create_subscription(Float64MultiArray,'/target_pos',self.target_position_callback,10)
        self.avoid_pos_sub = self.create_subscription(drone_msgs.msg.SphereArray,'/spheres',self.avoid_pos_callback,1)
        

        #Timer
        #Time step based on MPC Tf/N_horizon
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        #Set up real-time 3D plot
        plt.ion()  # Turn on interactive mode   
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  
        self.plot_x, self.plot_y, self.plot_z = [], [], []  
        self.scatter = self.ax.scatter([], [], [])
        self.ax.set_xlim(-10, 10)  
        self.ax.set_ylim(-10, 10)
        self.ax.set_zlim(0, 5) 
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_zlabel("Z Position")

        self.is_reached = Bool()
        self.is_reached.data = False
        self.total_time = 0
        self.error_tolerance = 0.3 # Error tolerance at the final goal position
        

    
    def avoid_pos_callback(self,msg):
        if msg is not None:
            positions = []
            radii = []           
            for sphere in msg.spheres:
                positions.append([sphere.position.x,sphere.position.y,sphere.position.z])
                radii.append(sphere.radius)
            self.avoid_positions = positions
            self.radius = radii

        new_avoid_pos = np.array(self.avoid_positions)

        if not np.array_equal(self.avoid_pos,new_avoid_pos):
            self.get_logger().info(f"Received new obstacles")

            self.avoid_pos = new_avoid_pos
            self.hover = True
            if self.hover:
                velocity_msg = Twist()
                velocity_msg.linear.x = 0.0
                velocity_msg.linear.y = 0.0
                velocity_msg.linear.z = 0.0

            self.velocity_pub.publish(velocity_msg)

            del self.drone_solver
            self.drone_solver = DroneMPCSolver()
            self.drone_solver.setup_solver(init_pos=self.initial_state, target_pos=self.target_pos,avoid_pos=self.avoid_pos,d_min=0.6)
            self.hover = False
            

    def current_pose_callback(self,msg):
        self.initial_state[:3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        distance  =np.linalg.norm(self.initial_state[:3] - self.target_pos[:3])

        if distance < self.error_tolerance and self.new_pos == True:
            self.get_logger().info("Target Reached")
            
            self.new_pos = False
            self.is_reached.data = not self.new_pos
            self.reached_point_pub.publish(self.is_reached)
            self.total_time = 0 # Reset total time after reaching the final position NOTE: comment it when computing for all waypoints within a run
            

    def target_position_callback(self,msg):
        
        #Update target position only if it is different from previous one

        new_target_pos = np.array(msg.data)

        if not np.array_equal(self.target_pos,new_target_pos):
            self.get_logger().info(f"Received new target position: {new_target_pos}")
            self.target_pos = np.array([new_target_pos[0],new_target_pos[1],new_target_pos[2],0.0,0.0,0.0])

            self.hover = True
            if self.hover:
                velocity_msg = Twist()
                velocity_msg.linear.x = 0.0
                velocity_msg.linear.y = 0.0
                velocity_msg.linear.z = 0.0

            self.velocity_pub.publish(velocity_msg)

            #Destory solver and reinitialize new one (must be a better way to do this)
            self.get_logger().info("Setting up new solver...")
            del self.drone_solver
            self.drone_solver = DroneMPCSolver()
            self.drone_solver.setup_solver(init_pos=self.initial_state, target_pos=self.target_pos,avoid_pos=self.avoid_pos,d_min=0.5)
            self.hover = False
            self.new_pos = True
            self.is_reached.data = not self.new_pos

    

    #🍞
    def timer_callback(self):

        #Solve optimization problem and get first control input
        start = timeit.default_timer()

        control_input, predicted_steps = self.drone_solver.solve(self.initial_state)

        stop = timeit.default_timer()

        if not self.is_reached.data: # Compute until reaching the final position
            time_taken_each_step = stop - start # Compute MPC computation time for each step
            self.total_time += time_taken_each_step # Accumulate MPC computation time

        self.get_logger().info(f"Target position: {self.target_pos}, Current state: {self.initial_state}, Total time: {self.total_time} s")
        self.visualize_steps(predicted_steps)

        if self.noise:
            noise_vel = np.random.normal(0, self.noise_std_vel, size=3)
        else:
            noise_vel = np.zeros(3)

        #Publish the velocity
        velocity_msg = Twist()
        velocity_msg.linear.x += (control_input[0] * self.dt) + noise_vel[0]
        velocity_msg.linear.y += (control_input[1] * self.dt) + noise_vel[1]
        velocity_msg.linear.z += (control_input[2] * self.dt) + noise_vel[2]

        if self.hover:
            velocity_msg.linear.x = 0.0
            velocity_msg.linear.y = 0.0
            velocity_msg.linear.z += 9.81

        self.velocity_pub.publish(velocity_msg)

        self.initial_state[3:] = control_input * self.dt

        # Store the new position for plotting
        self.positions.append(self.initial_state[0:3].copy())

        # # Update the 3D plot
        # #self.ax.clear()
        # self.ax.set_xlim(-10, 10)
        # self.ax.set_ylim(-10, 10)
        # self.ax.set_zlim(0, 10)
        # self.ax.set_xlabel("X Position")
        # self.ax.set_ylabel("Y Position")
        # self.ax.set_zlabel("Z Position")

        # # Plot the trajectory as a line
        # # Update the 3D plot
        # self.ax.cla()  
        # self.ax.set_xlim(-10, 10)
        # self.ax.set_ylim(-10, 10)
        # self.ax.set_zlim(0, 10)
        # self.ax.set_xlabel("X Position")
        # self.ax.set_ylabel("Y Position")
        # self.ax.set_zlabel("Z Position")

        # # Plot the trajectory as a line
        # x_vals = [pos[0] for pos in self.positions]
        # y_vals = [pos[1] for pos in self.positions]
        # z_vals = [pos[2] for pos in self.positions]
        # self.ax.plot(x_vals, y_vals, z_vals, c='b', marker='o')

        # if self.avoid_pos is not None:
        #     for pos in self.avoid_pos:
        #         self.ax.scatter(pos[0], pos[1], pos[2], c='r', marker='x')

        # # Redraw the plot and pause briefly
        # plt.draw()
        # plt.pause(0.1)

        # #Redraw the plot and pause briefly
        # plt.draw()
        # plt.pause(0.1)

    def visualize_steps(self, steps):
        # steps are a list of states
        msg = PoseArray()
        pose_list = []
        
        for state in steps:
            pos = state[:3]
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            pose_list.append(pose)
        
        msg.poses = pose_list
        self.waypoint_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    drone_mpc_node = DroneMPCNode()

    rclpy.spin(drone_mpc_node)

    # Destroy the node explicitly
    drone_mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

