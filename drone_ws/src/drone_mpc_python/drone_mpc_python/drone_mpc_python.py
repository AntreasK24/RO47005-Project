import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Float64MultiArray
import numpy as np
import transforms3d
from drone_mpc_python.mpcDroneSolver import DroneMPCSolver

class DroneMPCNode(Node):
    #Constructor
    def __init__(self):
        super().__init__('drone_mpc')


        self.Q = np.diag([10.0,10.0,10.0,4.0,4.0,4.0])
        self.R = np.diag([0.05,0.05,0.05])

        self.Q =  self.Q.flatten().tolist()
        self.R =  self.R.flatten().tolist()

        #Create MPC solver object
        self.drone_solver = DroneMPCSolver()

        self.declare_parameter('initial_state', [0.0,0.0,0.1125,0.0,0.0,0.0])
        self.declare_parameter('target_pos', [3.0, 3.0, 3.0, 0.0, 0.0, 0.0])

        self.declare_parameter('accel_max', 500)
        self.declare_parameter('N_horizon', 50)
        self.declare_parameter('prediction_period', 0.05)
        self.declare_parameter('Q',self.Q)
        self.declare_parameter('R', self.R)
        self.declare_parameter('drone_radius', 0.5)

        self.declare_parameter('d_min', 1.0)

        self.declare_parameter('noise', True)
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



        #Set initial state and default target position (this  could be a ROS param)
        self.initial_state = np.array(self.get_parameter('initial_state').value)
        self.target_pos = np.array(self.get_parameter('target_pos').value)

        #Setup solver
        self.drone_solver.setup_solver(init_pos=self.initial_state,target_pos=self.target_pos,avoid_pos=None,d_min=0.5)

        #Publisher
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Subscribers
        self.current_pose_sub = self.create_subscription(Pose,'/pose',self.current_pose_callback,10)
        self.target_position_sub = self.create_subscription(Float64MultiArray,'/target_pos',self.target_position_callback,10)

        #Timer
        #Time step based on MPC Tf/N_horizon
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def current_pose_callback(self,msg):
        self.initial_state[:3] = np.array([msg.position.x, msg.position.y, msg.position.z])

        

    def target_position_callback(self,msg):
        
        #Update target position only if it is different from previous one

        new_target_pos = np.array(msg.data)

        if not np.array_equal(self.target_pos,new_target_pos):
            self.get_logger().info(f"Received new target position: {new_target_pos}")
            self.target_pos = new_target_pos

            #Destory solver and reinitialize new one (must be a better way to do this)
            self.get_logger().info("Setting up new solver...")
            del self.drone_solver
            self.drone_solver = DroneMPCSolver()
            self.drone_solver.setup_solver(init_pos=self.initial_state, target_pos=self.target_pos,avoid_pos=None,d_min=0.5)

    #🍞
    def timer_callback(self):

        #Solve optimization problem and get first control input
        control_input = self.drone_solver.solve(self.initial_state)
        self.get_logger().info(f"Target position: {self.target_pos}, Current state: {self.initial_state}")
        

        if self.noise:
            noise_vel = np.random.normal(0, self.noise_std_vel, size=3)
            noise_pos = np.random.normal(0, self.noise_std_pos, size=3)
        else:
            noise_vel = np.zeros(3)
            noise_pos = np.zeros(3)

        #Publish the velocity
        velocity_msg = Twist()
        velocity_msg.linear.x += (control_input[0] * self.dt) + noise_vel[0]
        velocity_msg.linear.y += (control_input[1] * self.dt) + noise_vel[1]
        velocity_msg.linear.z += (control_input[2] * self.dt) + noise_vel[2]

        self.velocity_pub.publish(velocity_msg)

        self.initial_state[3:] = control_input * self.dt

def main(args=None):
    rclpy.init(args=args)

    drone_mpc_node = DroneMPCNode()

    rclpy.spin(drone_mpc_node)

    # Destroy the node explicitly
    drone_mpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

