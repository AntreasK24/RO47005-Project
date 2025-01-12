from casadi import *
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan
import numpy as np

def export_drone_ode_model(is_state_noise=False, is_input_noise=False) -> AcadosModel:
    model_name = 'drone_ode'
    
    # Constants (for the drone model)
    # Mass of the drone (kg)
    m = 0.027 
    
    # States: position and velocity in x, y, z
    x1 = SX.sym('x1')  # position in x
    x2 = SX.sym('x2')  # position in y
    x3 = SX.sym('x3')  # position in z
    v1 = SX.sym('v1')  # velocity in x
    v2 = SX.sym('v2')  # velocity in y
    v3 = SX.sym('v3')  # velocity in z

    x = vertcat(x1, x2, x3, v1, v2, v3)
    
    # Control inputs: acceleration in x, y, z
    ax = SX.sym('ax')
    ay = SX.sym('ay')
    az = SX.sym('az')
    
    u = vertcat(ax, ay, az)

    # Derivatives of states (xdot)
    x1_dot = SX.sym('x1_dot')
    x2_dot = SX.sym('x2_dot')
    x3_dot = SX.sym('x3_dot')
    v1_dot = SX.sym('v1_dot')
    v2_dot = SX.sym('v2_dot')
    v3_dot = SX.sym('v3_dot')

    xdot = vertcat(x1_dot, x2_dot, x3_dot, v1_dot, v2_dot, v3_dot)

    # Define system matrices A and B 
    A = np.array([
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ])
    
    B = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Define the state-space model as: x_dot = A * x + B * u
    f_expl = A @ x + B @ u
    f_expl[5] = f_expl[5] - m*9.81


    # Implicit dynamics (state derivative equals the dynamics)
    f_impl = xdot - f_expl  

    # Define the model
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    # Store labels for the state, control input, and time
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$v_x$ [m/s]', '$v_y$ [m/s]', '$v_z$ [m/s]']
    model.u_labels = ['$a_x$ [m/s²]', '$a_y$ [m/s²]', '$a_z$ [m/s²]']
    model.t_label = '$t$ [s]'

    return model
