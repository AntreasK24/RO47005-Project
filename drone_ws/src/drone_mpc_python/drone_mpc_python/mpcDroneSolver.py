from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca
import scipy.linalg
from drone_mpc_python.drone_model import export_drone_ode_model

class DroneMPCSolver:

    #Constructor
    def __init__(self,accel_max=500,N_horizon=200,prediction_period=0.05,Q=np.diag([10.0,10.0,10.0,4.0,4.0,4.0]),R=np.diag([0.05,0.05,0.05]),drone_radius = 0.5):        
        self.Q = Q 
        self.R = R 
        self.accel_max = accel_max
        self.N_horizon = N_horizon
        self.prediction_period = prediction_period
        self.ocp_solver = None
        self.drone_radius = drone_radius

    def setup_solver(self,init_pos,target_pos,avoid_pos,d_min):
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
        target_state = np.array([target_pos[0], target_pos[1], target_pos[2], 0.0, 0.0, 0.0])
        ocp.model.cost_y_expr = ca.vertcat(ocp.model.x - target_state, ocp.model.u)
        ocp.model.cost_y_expr_e = ocp.model.x - target_state
        ocp.cost.yref  = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros((ny_e, ))

        # Constraints
        ocp.constraints.lbu = np.array([-self.accel_max,-self.accel_max,-self.accel_max])
        ocp.constraints.ubu = np.array([+self.accel_max,+self.accel_max,+self.accel_max])
        ocp.constraints.lbx = np.array([-1000, -1000, -0.1]) 
        ocp.constraints.ubx = np.array([1000, 1000, 100]) 
        ocp.constraints.x0 = init_pos
        ocp.constraints.idxbu = np.array([0,1,2])
        ocp.constraints.idxbx = np.array([0, 1, 2])



        if ocp.model.con_h_expr is None:
            ocp.model.con_h_expr = ca.SX()
        
        if avoid_pos is not None:

            repulsion_term = 0
            lh = []
            uh = []


            for pos in avoid_pos:
                total_radius = self.drone_radius + (d_min) / 2
        
                # Distance from drone's position to obstacle
                dist_expr = ca.sumsqr(ocp.model.x[:3] - pos)
                distance_to_obstacle = ca.sqrt(dist_expr)  

                lh.append(d_min/2)
                uh.append(1000)
                ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, distance_to_obstacle)

                
                position_threshold = 10.5  # Threshold for repulsion to take effect
                
                # Create a switch that
                switch = ca.if_else(dist_expr < total_radius ** 2, 0, 1)
                
                # Repulsion term that activates when within the threshold
                repulsion_term += switch * 0.5 * 5 * ca.power((1 / distance_to_obstacle) - (1 / position_threshold), 2)

            # Add repulsion term to the cost expression
            ocp.model.cost_y_expr = ca.vertcat(ocp.model.cost_y_expr, repulsion_term)
            ocp.cost.yref = np.append(ocp.cost.yref, 1.0)

            ocp.constraints.lh = np.array(lh)
            ocp.constraints.uh = np.array(uh)

            repulsion_weight = np.array([20])

            ocp.cost.W = scipy.linalg.block_diag(self.Q, self.R, repulsion_weight)


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

        solver_json = 'acados_ocp_' + ocp.model.name + '.json'
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    
    def solve(self,init_pos):


        if self.ocp_solver is None:
            raise Exception('What are you doing? No do it again, but right this time')
        

        self.ocp_solver.solve_for_x0(x0_bar=init_pos)
        first_control_input = self.ocp_solver.get(0,"u")
        predicted_states = [self.ocp_solver.get(i, "x") for i in range(0,200, 20)]

        return first_control_input, predicted_states