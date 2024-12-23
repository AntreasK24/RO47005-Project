from casadi import *
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan
from scipy.spatial.transform import Rotation

def export_drone_ode_model(is_quaternion=True) -> AcadosModel:
    model_name = 'drone_non_linear_ode'
    
    # Constants (for the drone model)
    # Mass of the drone [kg]
    m = 0.027 #got this from urdf file 

    # Gravity [m/s^2]
    g = 9.81

    # Thrust factor
    b = 1 #dummy value

    # Drag factor
    d = 1 #dummy value

    # Distance b/w any rotor and the center of the drone
    l = 1 #dummy value

    # Inertia of the quadrotor in X,Y,Z axes
    Ix = 1e-3 #dummy value
    Iy = 1e-3 #dummy value
    Iz = 1e-3 #dummy value

    # States: {linear position, angular position} in world frame, {linear position, angular position} in body frame
    x = SX.sym('x')  # linear position in world frame --> x
    y = SX.sym('y')  # linear position in world frame --> y
    z = SX.sym('z')  # linear position in world frame --> z

    # Angular position in the world frame (in Euler angle representation)
    phi = SX.sym('phi')  # angular position in world frame --> phi Φ
    theta = SX.sym('theta')  # angular position in world frame --> theta θ
    psi = SX.sym('psi')  # angular position in world frame --> psi ψ

    u = SX.sym('u')  # linear position in body frame --> u
    v = SX.sym('v')  # linear position in body frame --> v
    w = SX.sym('w')  # linear position in body frame --> w

    # Angular position in the body frame (in Euler angle representation)
    p = SX.sym('p')  # angular position in body frame --> p
    q = SX.sym('q')  # angular position in body frame --> q
    r = SX.sym('r')  # angular position in body frame --> r

    # Angular positions (in Quaternion representation)
    if is_quaternion == True:
        w_euler_rot = Rotation.from_euler('xyz', [phi,theta,psi], degrees=False)
        w_qx,w_qy,w_qz,w_qw = w_euler_rot.as_quat()

        b_euler_rot = Rotation.from_euler('xyz', [p,q,r], degrees=False)
        b_qx,b_qy,b_qz,b_qw = b_euler_rot.as_quat()

        states = vertcat(x,y,z,w_qx,w_qy,w_qz,w_qw,u,v,w,b_qx,b_qy,b_qz,b_qw)
    else:
        states = vertcat(x,y,z,phi,theta,psi,u,v,w,p,q,r)

    
    # Control inputs: rotor angular velocities
    omega1 = SX.sym('omega1') # omega1
    omega2 = SX.sym('omega2') # omega2
    omega3 = SX.sym('omega3') # omega3
    omega4 = SX.sym('omega4') # omega4

    inputs = vertcat(omega1, omega2, omega3, omega4)

    # Derivatives of states (xdot)

    x_dot = SX.sym('x_dot')  # linear velocity in world frame --> x_dot
    y_dot = SX.sym('y_dot')  # linear velocity in world frame --> y_dot
    z_dot = SX.sym('z_dot')  # linear velocity in world frame --> z_dot

    phi_dot = SX.sym('phi_dot')  # angular velocity in world frame --> phi_dot Φ_dot
    theta_dot = SX.sym('theta_dot')  # angular velocity in world frame --> theta_dot θ_dot
    psi_dot = SX.sym('psi_dot')  # angular velocity in world frame --> psi_dot ψ_dot

    u_dot = SX.sym('u_dot')  # linear velocity in body frame --> u_dot
    v_dot = SX.sym('v_dot')  # linear velocity in body frame --> v_dot
    w_dot = SX.sym('w_dot')  # linear velocity in body frame --> w_dot

    p_dot = SX.sym('p_dot')  # angular velocity in body frame --> p_dot
    q_dot = SX.sym('q_dot')  # angular velocity in body frame --> q_dot
    r_dot = SX.sym('r_dot')  # angular velocity in body frame --> r_dot

    # Angular velocities (in Quaternion representation)
    if is_quaternion == True:
        w_euler_rot = Rotation.from_euler('xyz', [phi,theta,psi], degrees=False)
        w_qx,w_qy,w_qz,w_qw = w_euler_rot.as_quat()

        b_euler_rot = Rotation.from_euler('xyz', [p,q,r], degrees=False)
        b_qx,b_qy,b_qz,b_qw = b_euler_rot.as_quat()

        states = vertcat(x,y,z,w_qx,w_qy,w_qz,w_qw,u,v,w,b_qx,b_qy,b_qz,b_qw)
    else:
        states = vertcat(x,y,z,phi,theta,psi,u,v,w,p,q,r)

    xdot = vertcat(x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot)

    # Compute trignometric angles
    sin_phi = sin(phi)
    cos_phi = cos(phi)

    sin_theta = sin(theta)
    cos_theta = cos(theta)
    tan_theta = tan(theta)

    sin_psi = sin(psi)
    cos_psi = cos(psi)
    
    # Control inputs: thrust, torque in {x,y,z} body frame axes
    ft = b*(omega1**2+omega2**2+omega3**2+omega4**2) # thrust force in z axis (up w.r.t body frame is positive)
    tau_x = b*l*(omega3**2 - omega1**2) # torque in x axis
    tau_y = b*l*(omega4**2 - omega2**2) # torque in y axis
    tau_z = d*(omega2**2+omega4**2-omega1**2-omega3**2) # torque in z axis

    # Define change in states (non-linear dynamics equations)

    f_expl = vertcat(w*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - v*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + u*(cos_psi*cos_theta),
                     v*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - w*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + u*(cos_theta*cos_psi),
                     w*(cos_phi*cos_theta) -u*(sin_theta) + v*(cos_theta*sin_phi),

                     p + r*(cos_phi*tan_theta) + q(sin_phi*tan_theta),
                     q*(cos_phi) - r*(sin_phi),
                     r*(cos_phi/cos_theta) + q*(sin_phi/cos_theta),

                     r*v - q*w - g*(sin_theta),
                     p*w - r*u + g*(sin_phi*cos_theta),
                     q*u - p*v + g*(cos_theta*cos_phi) - (ft/m),

                    ((Iy-Iz)/Ix)*(r*q) + (tau_x/Ix),
                    ((Iz-Ix)/Iy)*(p*r) + (tau_y/Iy),
                    ((Ix-Iy)/Iz)*(p*q) + (tau_z/Iz)

    )

    f_impl = xdot - f_expl  # Implicit dynamics (state derivative equals the dynamics)

    # Define the model
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = xdot
    model.u = inputs
    model.name = model_name

    # Store labels for the state, control input, and time
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$z$ [m]', '$phi$ [rad/s]', '$theta$ [rad/s]', '$psi$ [rad/s]', '$u$ [m]', '$v$ [m]', '$w$ [m]', '$p$ [rad/s]', '$q$ [rad/s]' '$r$ [rad/s]']
    model.u_labels = ['$omega1$ [rad/s]', '$omega2$ [rad/s]', '$omega3$ [rad/s]', '$omega4$ [rad/s]']
    model.t_label = '$t$ [s]'

    return model