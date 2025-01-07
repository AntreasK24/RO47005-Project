from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
import scipy.linalg
from drone_mpc_python.drone_model import export_drone_ode_model, b , m , g

class DroneMPCSolver:

    #Constructor
    def __init__(self,accel_max=10,N_horizon=40,prediction_period=0.8,Q=np.diag([10.0,10.0,10.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]),R=np.diag([1.0,1.0,1.0,1.0])):

        self.Q = Q if not None else np.diag([10.0,10.0,10.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0])
        self.R = R if not None else np.diag([1.0,1.0,1.0,1.0])
        self.accel_max = accel_max
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
        target_state = np.array(target_pos)
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        ocp.cost.yref  = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros((ny_e, ))

        # Constraints
        # ocp.constraints.lbu = np.array([-self.accel_max])
        # ocp.constraints.ubu = np.array([+self.accel_max])
        ocp.constraints.x0 = init_pos
        # ocp.constraints.idxbu = np.array([0])
        ocp.constraints.lbu = -self.accel_max * np.ones((nu,))
        ocp.constraints.ubu = self.accel_max * np.ones((nu,))
        ocp.constraints.idxbu = np.arange(nu)

        # Set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.prediction_period

        # Solver stuff
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'IRK'
        ocp.solver_options.sim_method_newton_iter = 10

        ocp.solver_options.nlp_solver_tol_stat = 1e-6
        ocp.solver_options.nlp_solver_tol_eq = 1e-6

        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.nlp_solver_max_iter = 250

        ocp.solver_options.qp_solver_cond_N = self.N_horizon

        solver_json = 'acados_ocp_' + ocp.model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    
    def solve(self,init_pos,target_pos):

        #Solve optimization problem and return first control input (acceleration)

        if self.ocp_solver is None:
            raise Exception('What are you doing? No do it again, but right this time')
        
        ocp = self.ocp_solver.acados_ocp
        ocp.constraints.x0 = init_pos
        target_state = np.array(target_pos)
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        self.ocp_solver.solve_for_x0(x0_bar=self.init_pos)

        x_trajectory = self.ocp_solver.get(0,"x")
        first_control_input = self.ocp_solver.get(0,"u")

        phi = x_trajectory[3]
        theta = x_trajectory[4]

        print(first_control_input)

        #Convert from actuator commands to acceleration

        thrust = b*(first_control_input[0]**2+first_control_input[1]**2+first_control_input[2]**2+first_control_input[3]**2)

        acceleration_z = -(thrust - m * g)/m

        acceleration_x = (thrust/m)*np.sin(theta)
        acceleration_y = (thrust/m)*np.sin(phi)

        accelerations = np.array([acceleration_x,acceleration_y,acceleration_z])


        return accelerations