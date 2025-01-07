from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
import scipy.linalg
from drone_mpc_python.drone_model import export_drone_ode_model

# Gravitational acceleration
g_const = 9.8124  # m/s^2

class DroneMPCSolver:

    #Constructor
    def __init__(self,accel_max=10,N_horizon=20,prediction_period=1.0,Q=np.diag([100, 100, 100, 1, 1, 1, 1, 1, 1, 10, 10, 10]),R=np.diag([1, 1, 1, 1])):

        self.Q = Q if not None else np.diag([100, 100, 100, 1, 1, 1, 1, 1, 1, 10, 10, 10])
        self.R = R if not None else np.diag([1, 1, 1, 1])
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

        # dimensions
        nx = ocp.model.x.rows()
        nu = ocp.model.u.rows()
        ny = nx + nu
        ny_e = nx

        mass_value = 0.752  # kg
        q_ref_init = np.array([1.0, 0.0, 0.0, 0.0])
        cd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        l_x = np.array([0.075, -0.075, -0.075, 0.075])
        l_y = np.array([-0.10, 0.10, -0.10, 0.10])
        kappa = 0.022
        inertia_diag = np.array([0.0025, 0.0021, 0.0043])
        ocp.parameter_values = np.concatenate(
            [
                np.array([mass_value]),
                q_ref_init,
                cd,
                l_x,
                l_y,
                np.array([kappa]),
                inertia_diag,
            ]
        )

        # Constraints
        omega_max = [10.0, 10.0, 4.0]  # [rad/s]
        thrust_min = 0.0  # [N]
        thrust_max = 8.5  # [N] per motor

        Jbx = np.zeros((3, nx))
        Jbx[0, 10] = 1.0
        Jbx[1, 11] = 1.0
        Jbx[2, 12] = 1.0
        ocp.constraints.Jbx = Jbx
        ocp.constraints.lbx = -10 * np.ones((3,))
        ocp.constraints.ubx = 10 * np.ones((3,))

        Jbu = np.identity(nu)
        ocp.constraints.Jbu = Jbu
        ocp.constraints.lbu = thrust_min * np.ones((nu,))
        ocp.constraints.ubu = thrust_max * np.ones((nu,))  

        # Set up cost
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'

        ocp.cost.W = scipy.linalg.block_diag(self.Q, self.R)
        ocp.cost.W_e = self.Q

        # initial references
        hover_prop = mass_value * g_const / 4.0
        ocp.cost.yref = np.array(
            [
                0,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                hover_prop,
                hover_prop,
                hover_prop,
                hover_prop,
            ]
        )
        ocp.cost.yref_e = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])       

        # Define the reference (desired) target in the cost
        target_state = np.array([target_pos[0], target_pos[1], target_pos[2], 0.0, 0.0, 0.0])
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        ocp.cost.yref  = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros((ny_e, ))

        # Constraints
        # ocp.constraints.lbu = np.array([-self.accel_max])
        # ocp.constraints.ubu = np.array([+self.accel_max])
        ocp.constraints.x0 = init_pos
        ocp.constraints.idxbu = np.array([0])

        # Set prediction horizon
        ocp.solver_options.N_horizon = self.N_horizon
        ocp.solver_options.tf = self.prediction_period

        # Solver stuff
        Tf =1.0
        ocp.solver_options.tf = Tf
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # "PARTIAL_CONDENSING_HPIPM", "FULL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'  # "GAUSS_NEWTON", "EXACT"
        ocp.solver_options.integrator_type = "ERK"  # "ERK", "IRK", "GNSF"
        # ocp.solver_options.sim_method_newton_iter = 10

        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # "SQP", "SQP_RTI"
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        # ocp.solver_options.nlp_solver_max_iter = 150

        ocp.solver_options.qp_solver_cond_N = self.N_horizon

        solver_json = 'acados_ocp_' + ocp.model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    
    def solve(self,init_pos,target_pos):
        #Solve optimization problem and return first control input (acceleration)

        if self.ocp_solver is None:
            raise Exception('What are you doing? No do it again, but right this time')
        
        ocp = self.ocp_solver.acados_ocp
        ocp.constraints.x0 = init_pos
        target_state = np.array([target_pos[0], target_pos[1], target_pos[2], 0.0, 0.0, 0.0])
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        self.ocp_solver.solve_for_x0(x0_bar=self.init_pos)
        first_control_input = self.ocp_solver.get(0,"u")

        # x1 = self.ocp_solver.get(1, "x")
        # self.ocp_solver.set(0, "lbx", x1)
        # self.ocp_solver.set(0, "ubx", x1)

        return first_control_input