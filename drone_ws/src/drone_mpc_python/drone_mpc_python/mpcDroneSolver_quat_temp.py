from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
import scipy.linalg
from drone_model_quat import export_drone_ode_model


def quat_mult(q1, q2):
    # Quaternion multiplication: q1 * q2
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  # w
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  # x
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  # y
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2   # z
    ])

def rotate_quat(q1, v1):
    """
    Rotate vector v1 using quaternion q1 (numeric version).

    Args:
    - q1: Quaternion (4-element numpy array)
    - v1: 3D vector to rotate (3-element numpy array)

    Returns:
    - Rotated vector (3-element numpy array)
    """
    #print("========",v1)

    # Step 1: Represent v1 as a pure quaternion with 0 as the real part
    v1_quat = np.array([0,*v1])  # [0, v1_x, v1_y, v1_z]

    # print("========",v1_quat)
    
    # Step 2: Quaternion multiplication q1 * v1_quat
    temp = quat_mult(q1, v1_quat)
    
    # Step 3: Quaternion multiplication result * conjugate of q1
    q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])  # Conjugate of q1
    ans = quat_mult(temp, q1_conj)
    
    # Return only the vector part of the result (discard the real part)
    return ans[1:]  # [x, y, z]


class DroneMPCSolver:

    #Constructor
    def __init__(self,thrust_max=10,N_horizon=100,prediction_period=0.001,Q=np.diag([10, 10, 10, 1, 1, 1, 1, 10, 10, 10, 1, 1, 1]),R=np.diag([1.0,1.0,1.0, 1.0])):

        self.Q = Q if not None else np.diag([100, 100, 100, 1, 1, 1, 10, 10, 10, 1, 1, 1])
        self.R = R if not None else np.diag([1.0,1.0,1.0, 1.0])
        self.thrust_max = thrust_max
        self.N_horizon = N_horizon
        self.prediction_period = prediction_period
        self.ocp_solver = None

    def setup_solver(self,init_pos,target_pos):
        # Create OCP object
        ocp = AcadosOcp()
        self.init_pos = init_pos

        model = export_drone_ode_model()
        ocp.model = model

        nx = ocp.model.x.rows()
        nu = ocp.model.u.rows()
        ny = nx + nu
        ny_e = nx

        # Set up cost
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'

        ocp.cost.W = scipy.linalg.block_diag(self.Q, self.R)
        ocp.cost.W_e = self.Q

        # Define the reference (desired) target in the cost
        target_state = np.array([target_pos[0], target_pos[1], target_pos[2], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        ocp.cost.yref  = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros((ny_e, ))

        # Constraints
        ocp.constraints.lbu = np.array([-self.thrust_max])
        ocp.constraints.ubu = np.array([+self.thrust_max])
        ocp.constraints.x0 = init_pos
        ocp.constraints.idxbu = np.array([0])
        

        # Set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.prediction_period

        # Solver stuff
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'  
        ocp.solver_options.sim_method_newton_iter = 100

        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.nlp_solver_max_iter = 250

        ocp.solver_options.qp_solver_cond_N = self.N_horizon

        solver_json = 'acados_ocp_' + ocp.model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    
    def solve(self,init_pos):


        if self.ocp_solver is None:
            raise Exception('What are you doing? No do it again, but right this time')
        

        self.ocp_solver.solve_for_x0(x0_bar=init_pos)
        first_control_input = self.ocp_solver.get(0,"u")
        states = self.ocp_solver.get(1,"x")
        
        #print(first_control_input)
        T1,T2,T3,T4 = first_control_input
        T_sum = np.array([0,0,np.sum(first_control_input)])
        #print(T_sum)
        #print(rotate_quat(states[3:7].reshape(4,1), first_control_input).shape)
        #print(np.vstack([0,0,-9.81]).shape)
        acc = np.vstack([0,0,0]) + (1/0.027 )*rotate_quat(states[3:7].reshape(4,1),  T_sum)
        
        acc_x , acc_y , acc_z = acc

        acc=np.array([acc_x,acc_z,acc_y]).reshape(3)

        return acc