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

<<<<<<< HEAD
    #Gravity [m/s^2]
    g = 0

    #Drag coefficient
    drag_coeff = 0.0

    #Inertia
    Ix = 0.001
    Iy = 0.001
    Iz = 0.001

    # Noise parameters
    mu_states = 0.5
    mu_inputs = 0.5

    #States
    x = SX.sym('x')
    y = SX.sym('y')
    z = SX.sym('z')

    k = SX.sym('u')
    v = SX.sym('v')
    w = SX.sym('w')

    phi = SX.sym('phi')
    theta = SX.sym('theta')
    psi = SX.sym('psi')

    p = SX.sym('p')
    q = SX.sym('q')
    r = SX.sym('r')

    x = vertcat(x,y,z,k,v,w,phi,theta,psi,p,q,r)

    #Control Input
    ax = SX.sym('ax')
    ay = SX.sym('ay')
    az = SX.sym('az')
    p_dot = SX.sym('p_dot')
    q_dot = SX.sym('q_dot')
    r_dot = SX.sym('r_dot')

    u = vertcat(ax,ay,az,p_dot,q_dot,r_dot)

    #derivatives of state
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    k_dot = SX.sym('k_dot')
    v_dot = SX.sym('v_dot')
    w_dot = SX.sym('w_dot')
    psi_dot = SX.sym('psi_dot')
    theta_dot = SX.sym('theta_dot')
    phi_dot = SX.sym('phi_dot')
    p_dot_dyn = SX.sym('p_dot_dyn')
    q_dot_dyn = SX.sym('q_dot_dyn')
    r_dot_dyn = SX.sym('r_dot_dyn')

    xdot = vertcat(x_dot, y_dot, z_dot, k_dot, v_dot, w_dot, psi_dot, theta_dot, phi_dot, p_dot_dyn, q_dot_dyn, r_dot_dyn)

    #I do not remember why I defined all this (I will keep it here for now)
    acc = vertcat(ax, ay, az)
    F_gravity = vertcat(0, 0, -m*g)
    F_drag = vertcat(
        -drag_coeff * k**2,  # drag in x
        -drag_coeff * v**2,  # drag in y
        -drag_coeff * w**2   # drag in z
    )
    thrust = m * g + m * (ax**2 + ay**2 + az**2)
    I = vertcat(Ix, Iy, Iz)
    moment = vertcat(
        Ix * p_dot + (Iz - Iy) * q * r,
        Iy * q_dot + (Ix - Iz) * p * r,
        Iz * r_dot + (Iy - Ix) * p * q
    )

    #Dynamics of translational movment 
    f_trans = vertcat(k, v, w)
    v_dot = vertcat(
        ax - (drag_coeff * k) / m,
        ay - (drag_coeff * v) / m,
        az - (drag_coeff * w - g) / m
    )

    #This isn't a force 
    f_rot = vertcat(
        p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta),  # phi dot
        q*cos(phi) - r*sin(phi),  # theta dot
        (q*sin(phi) + r*cos(phi)) / cos(theta)  # psi dot
    )
=======
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
>>>>>>> 31234f3 (Added obstacle avoidance)

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

<<<<<<< HEAD
    # With Additive gaussian noise in measurements and actuators
    if is_state_noise == True:
        state_dims = len(x_dot)
        x_dot = x_dot + mu_states*np.random.multivariate_normal(mean=np.zeros(state_dims), cov=np.eye(state_dims), size=state_dims).T #  x_dot = x_dot + state_noise_weight*(standard multi-variate normal distribution) i.e., zero mean and unit standart deviation
        
    if is_input_noise == True:
        input_dims = len(inputs)
        inputs = inputs + mu_inputs*np.random.multivariate_normal(mean=np.zeros(input_dims), cov=np.eye(input_dims), size=input_dims).T #  inputs = inputs + input_noise_weight*(standard multi-variate normal distribution) i.e., zero mean and unit standart deviation
=======
    # Define the state-space model as: x_dot = A * x + B * u
    f_expl = A @ x + B @ u

    f_impl = xdot - f_expl  # Implicit dynamics (state derivative equals the dynamics)
>>>>>>> 31234f3 (Added obstacle avoidance)

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
