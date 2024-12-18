from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
import scipy.linalg
from drone_mpc_python.drone_model import export_drone_ode_model

class DroneMPCSolver:

    # Constructor
    def __init__(self, accel_max=10, N_horizon=40, prediction_period=0.8,
                Q = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
                R = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])):
        self.Q = Q if Q is not None else np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.R = R if R is not None else np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.accel_max = accel_max
        self.N_horizon = N_horizon
        self.prediction_period = prediction_period
        self.ocp_solver = None

    def setup_solver(self, init_pos, target_pos):
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
        ocp.constraints.lbu = np.array([-self.accel_max, -self.accel_max, -self.accel_max, -10000.0, -10000.0, -10000.0])
        ocp.constraints.ubu = np.array([self.accel_max, self.accel_max, self.accel_max, 10000.0, 10000.0, 10000.0])
        ocp.constraints.x0 = init_pos
        ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5]) 

        # Set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.prediction_period

        # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'IRK'
        ocp.solver_options.sim_method_newton_iter = 10
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.nlp_solver_max_iter = 150
        ocp.solver_options.qp_solver_cond_N = self.N_horizon

        # Solver JSON file
        solver_json = 'acados_ocp_' + ocp.model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    def solve(self, init_pos, target_pos):
        if self.ocp_solver is None:
            raise Exception('Solver not set up. Call setup_solver first.')
        
        ocp = self.ocp_solver.acados_ocp
        ocp.constraints.x0 = init_pos
        target_state = np.array(target_pos)
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        
        # Solve the optimization problem
        self.ocp_solver.solve_for_x0(x0_bar=init_pos)
        first_control_input = self.ocp_solver.get(0, "u")
        return first_control_input