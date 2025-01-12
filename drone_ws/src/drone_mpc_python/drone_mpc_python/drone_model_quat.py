import casadi as ca
from acados_template import AcadosModel
import numpy as np
from utils import quat_error, quat_mult, rotate_quat

def export_drone_ode_model(is_state_noise=False, is_input_noise=False) -> AcadosModel:
    model_name = 'drone_ode'
    
    # Constants (for the drone model)
    # Mass of the drone (kg)
    m = 0.027 
    
    # States: position, quarternion, velocity and body rates in x, y, z
    px = ca.SX.sym('px')  # position in x
    py = ca.SX.sym('py')  # position in y
    pz = ca.SX.sym('pz')  # position in z

    qw = ca.SX.sym('qw')  # quaternion qw
    qx = ca.SX.sym('qx')  # quaternion qx
    qy = ca.SX.sym('qy')  # quaternion qy
    qz = ca.SX.sym('qz')  # quaternion qz

    vx = ca.SX.sym('vx')  # velocity in x
    vy = ca.SX.sym('vy')  # velocity in y
    vz = ca.SX.sym('vz')  # velocity in z

    wx = ca.SX.sym('wx')  # body rates in x
    wy = ca.SX.sym('wy')  # body rates in y
    wz = ca.SX.sym('wz')  # body rates in z

    #States
    x = ca.vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)

    #Gravity [m/s^2]
    g = 9.81

    #Drag coefficient
    drag_coeff = 0.0

    #Inertia
    Ix = 0.0025
    Iy = 0.0021
    Iz = 0.0043

    J =  ca.SX.eye(3)
    J[0,0] = Ix
    J[1,1] = Iy
    J[2,2] = Iz

    J_inv = ca.SX.eye(3)
    J_inv[0, 0] = 1 / Ix
    J_inv[1, 1] = 1 / Iy
    J_inv[2, 2] = 1 / Iz

    # Noise parameters
    mu_states = 0.5
    mu_inputs = 0.5

    #Control Input
    T1 = ca.SX.sym('T1')
    T2 = ca.SX.sym('T2')
    T3 = ca.SX.sym('T3')
    T4 = ca.SX.sym('T4')

    u = ca.vertcat(T1, T2, T3, T4)

    l = 0.1 #arm length
    c_tau = 0.022 #rotor's torque constant

    #derivatives of state
    px_dot = ca.SX.sym('px_dot')  # velocity in x
    py_dot = ca.SX.sym('py_dot')  # velocity in y
    pz_dot = ca.SX.sym('pz_dot')  # velocity in z

    qw_dot = ca.SX.sym('qw_dot')  # time derivative of quaternion qw
    qx_dot = ca.SX.sym('qx_dot')  # time derivative of quaternion qx
    qy_dot = ca.SX.sym('qy_dot')  # time derivative of quaternion qy
    qz_dot = ca.SX.sym('qz_dot')  # time derivative of quaternion qz

    vx_dot = ca.SX.sym('vx_dot')  # acceleration in x
    vy_dot = ca.SX.sym('vy_dot')  # acceleration in y
    vz_dot = ca.SX.sym('vz_dot')  # acceleration in z

    wx_dot = ca.SX.sym('wx_dot')  # time derivative of body rates in x
    wy_dot = ca.SX.sym('wy_dot')  # time derivative of body rates in y
    wz_dot = ca.SX.sym('wz_dot')  # time derivative of body rates in z

    xdot = ca.vertcat(px_dot, py_dot, pz_dot, qw_dot, qx_dot, qy_dot, qz_dot, vx_dot, vy_dot, vz_dot, wx_dot, wy_dot, wz_dot)

    l_x = np.array([0.075, -0.075, -0.075, 0.075])
    l_y =  np.array([-0.10, 0.10, -0.10, 0.10])

    t_BM = ca.horzcat(-l_x, l_y)

    tau_yx = ca.mtimes(t_BM.T, u)

    # Dynamics
    f_expl = ca.vertcat(
        vx, 
        vy,
        vz, # p_dot = velocity in 3D
        0.5*quat_mult(ca.vertcat(qw,qx,qy,qz),ca.vertcat(0,wx,wy,wz)), # time derivative of quaternion (q_dot = 0.5* quat_mult(quat, (0,body_rates)))
        ca.DM([0, 0, -g]) + rotate_quat(ca.vertcat(qw,qx,qy,qz), ca.vertcat(0,0, T1/m, T2/m, T3/m, T4/m)), # velocity_dot = g + (1/m)*(quat_rot(q, (0,0,rotors_thrust))
        # ca.mtimes(J_inv,
        #           (ca.vertcat((l/np.sqrt(2))*(T1+T2-T3-T4),
        #                       (l/np.sqrt(2))*(-T1+T2+T3-T4), 
        #                       c_tau*(T1-T2+T3-T4)) 
        #             - ca.cross(ca.vertcat(wx,wy,wz),ca.mtimes(J, ca.vertcat(wx,wy,wz))))) # time derivative of body rates = inertial_inv @ ( torques - cross_product(body_rates, (inertia @ body_rates)))
        ca.mtimes(
            J_inv,
            ca.vertcat(  # w _dot
                tau_yx[1], tau_yx[0], (-T1 - T2 + T3 + T4)
            )
            - ca.cross(ca.vertcat(wx,wy,wz), ca.mtimes(J, ca.vertcat(wx,wy,wz))),
    ))

    # With Additive gaussian noise in measurements and actuators
    if is_state_noise == True:
        state_dims = len(x_dot)
        x_dot = x_dot + mu_states*np.random.multivariate_normal(mean=np.zeros(state_dims), cov=np.eye(state_dims), size=state_dims).T #  x_dot = x_dot + state_noise_weight*(standard multi-variate normal distribution) i.e., zero mean and unit standart deviation
        
    if is_input_noise == True:
        input_dims = len(inputs)
        inputs = inputs + mu_inputs*np.random.multivariate_normal(mean=np.zeros(input_dims), cov=np.eye(input_dims), size=input_dims).T #  inputs = inputs + input_noise_weight*(standard multi-variate normal distribution) i.e., zero mean and unit standart deviation

    # Define the model
    model = AcadosModel()


    model.x = x
    model.xdot = xdot
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.u = u
    model.name = model_name

    # Store labels for the state, control input, and time
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$qw$ [no_unit]', '$qx$ [no_unit]', '$qy$ [no_unit]', '$qz$ [no_unit]', '$vx$ [m/s]', '$vy$ [m/s]', '$vz$ [m/s]', '$wx$ [rad/s]', '$wy$ [rad/s]', '$wz$ [rad/s]']
    model.u_labels = ['$T1$ [N]', '$T2$ [N]', '$T3$ [N]', '$T4$ [N]']
    model.t_label = '$t$ [s]'

    return model    