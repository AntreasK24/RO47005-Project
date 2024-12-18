from casadi import *
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_drone_ode_model() -> AcadosModel:
    model_name = 'drone_ode'
    
    # Constants (for the drone model)
    # Mass of the drone [kg]
    m = 0.027 #got this from urdf file 

    #Gravity [m/s^2]
    g = 0

    #Drag coefficient
    drag_coeff = 0.0

    #Inertia
    Ix = 0.001
    Iy = 0.001
    Iz = 0.001


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

    acc = vertcat(ax, ay, az)

    F_gravity = vertcat(0, 0, -m*g)

    F_drag = vertcat(
        -drag_coeff * k**2,  # drag in x
        -drag_coeff * v**2,  # drag in y
        -drag_coeff * w**2   # drag in z
    )

    f_trans = vertcat(k, v, w)
    thrust = m * g + m * (ax**2 + ay**2 + az**2)

    v_dot = vertcat(
        (ax - drag_coeff * k) / m,
        (ay - drag_coeff * v) / m,
        (az - drag_coeff * w - g) / m
    )

    I = vertcat(Ix, Iy, Iz)

    moment = vertcat(
        Ix * p_dot + (Iz - Iy) * q * r,
        Iy * q_dot + (Ix - Iz) * p * r,
        Iz * r_dot + (Iy - Ix) * p * q
    )

    f_rot = vertcat(
        p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta),  # phi dot
        q*cos(phi) - r*sin(phi),  # theta dot
        (q*sin(phi) + r*cos(phi)) / cos(theta)  # psi dot
    )

    

    f_impl = xdot - vertcat(f_trans, v_dot, f_rot,moment)

    # Define the model
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = vertcat(f_trans, v_dot, f_rot,moment)
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    # Store labels for the state, control input, and time
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$v_x$ [m/s]', '$v_y$ [m/s]', '$v_z$ [m/s]',
                    '$\\psi$ [rad]', '$\\theta$ [rad]', '$\\phi$ [rad]', '$p$ [rad/s]', '$q$ [rad/s]', '$r$ [rad/s]']
    model.u_labels = ['$a_x$ [m/s²]', '$a_y$ [m/s²]', '$a_z$ [m/s²]', '$p_{dot}$ [rad/s²]', '$q_{dot}$ [rad/s²]', '$r_{dot}$ [rad/s²]']
    model.t_label = '$t$ [s]'

    return model





