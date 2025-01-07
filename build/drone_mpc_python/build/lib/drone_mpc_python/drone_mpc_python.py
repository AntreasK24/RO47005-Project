import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
import numpy as np
from drone_mpc_python.mpcDroneSolver import DroneMPCSolver

class DroneMPCNode(Node):

    mass = 0.027 #kg
    gravity = 9.81 #m/s^2
    thrust_factor = 1

    #Constructor
    def __init__(self):
        super().__init__('drone_mpc')

        #Create MPC solver object
        self.drone_solver = DroneMPCSolver()

        #Set initial state and default target position (this  could be a ROS param)
        # self.initial_state = np.array([0.0,0.0,0.1125,0.0,0.0,0.0])
        # self.target_pos = np.array([1.0,1.0,1.0])

        self.initial_state = np.zeros(12)
        self.initial_state[2] = 0.1125

        self.target_pos = np.zeros(12)
        self.target_pos[0] = 1.0
        self.target_pos[1] = 1.0
        self.target_pos[2] = 1.0

        #Setup solver
        self.drone_solver.setup_solver(init_pos=self.initial_state,target_pos=self.target_pos)

        #Publisher
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Subscribers
        self.current_pose_sub = self.create_subscription(Pose,'/pose',self.current_pose_callback,10)
        self.target_position_sub = self.create_subscription(Point,'/target_pos',self.target_position_callback,10)

        #Timer
        #Time step based on MPC Tf/N_horizon
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def current_pose_callback(self,msg):
        self.initial_state[:3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.initial_state[3:6] = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z]) 

    def target_position_callback(self,msg):
        
        #Update target position only if it is different from previous one

        new_target_pos = np.array([msg.x,msg.y,msg.z])

        if not np.array_equal(self.target_pos,new_target_pos):
            self.get_logger().info(f"Received new target position: {new_target_pos}")
            self.target_pos = new_target_pos

            #Destory solver and reinitialize new one (must be a better way to do this)
            self.get_logger().info("Setting up new solver...")
            del self.drone_solver
            self.drone_solver = DroneMPCSolver()
            self.drone_solver.setup_solver(init_pos=self.initial_state, target_pos=self.target_pos)

    #🍞
    def timer_callback(self):

        #Solve optimization problem and get first control input
        control_input = self.drone_solver.solve(self.initial_state, self.target_pos)


        #self.get_logger().info(f"Target position: {self.target_pos}, Current position: {self.initial_state[:3]}, Velocity: {self.initial_state[3:]}")
        
        #Publish the velocity
        velocity_msg = Twist()
        velocity_msg.linear.x += control_input[0] * self.dt
        velocity_msg.linear.y += control_input[1] * self.dt
        velocity_msg.linear.z += control_input[2] * self.dt

        self.velocity_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)

    drone_mpc_node = DroneMPCNode()

    rclpy.spin(drone_mpc_node)

    # Destroy the node explicitly
    drone_mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

