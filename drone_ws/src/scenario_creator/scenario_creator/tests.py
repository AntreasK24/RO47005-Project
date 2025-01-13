import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import drone_msgs.msg  # Assuming you have this package available (if not, substitute with a custom class)
import geometry_msgs.msg as geom_msgs  # Import geometry_msgs for Point

# Define the mock pose class and the Quaternion-to-Rotation function
class Pose:
    def __init__(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        self.position = Position(x, y, z)
        self.orientation = Orientation(quat_x, quat_y, quat_z, quat_w)

class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

def quaternion_to_rotation_matrix(orientation):
    # This function converts the quaternion into a rotation matrix.
    q0, q1, q2, q3 = orientation
    rotation_matrix = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    return rotation_matrix

class DroneSimulator:
    def __init__(self):
        pass

    def cuboid_to_sphere(self, pose, x_len, y_len, z_len):
        spheres = []
        # Calculate the desired positions and radius of the spheres
        center_position = np.array([[pose.position.x], [pose.position.y], [pose.position.z]])

        # Get the orientation
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = quaternion_to_rotation_matrix(orientation)

        base_position = center_position - rotation_matrix.dot(np.array([[x_len], [y_len], [z_len]]))

        # Get the size
        size = [x_len, y_len, z_len]
        r_obj = 1.0 if min(size) >= 1.0 else (0.3 if min(size) <= 0.3 else min(size))
        r_sphere = np.sqrt(r_obj**2 + r_obj**2)  # Sphere radius - will be 1.41 * radius_cylinder

        # Determine the number of spheres along each axis
        num_spheres_x = int(x_len // r_obj)
        num_spheres_y = int(y_len // r_obj)
        num_spheres_z = int(z_len // r_obj)

        # Generate 3D meshgrid of positions
        x_positions = np.linspace(base_position[0, 0], base_position[0, 0] + x_len, num_spheres_x)
        y_positions = np.linspace(base_position[1, 0], base_position[1, 0] + y_len, num_spheres_y)
        z_positions = np.linspace(base_position[2, 0], base_position[2, 0] + z_len, num_spheres_z)

        # Create the meshgrid
        Xs, Ys, Zs = np.meshgrid(x_positions, y_positions, z_positions)

        Xs = Xs.flatten()
        Ys = Ys.flatten()
        Zs = Zs.flatten()

        for i in range(len(Xs)):
            sphere = drone_msgs.msg.Sphere()  # Assuming this is a simple class or substitute for testing
            sphere_position = np.array([Xs[i], Ys[i], Zs[i]])

            # Apply the rotation to the spheres
            rotated_position = rotation_matrix.dot(sphere_position - center_position)
            rotated_position = rotated_position + center_position

            # Create a geometry_msgs/Point to set the position of the sphere
            sphere_position_msg = geom_msgs.Point()
            sphere_position_msg.x = float(rotated_position.flatten()[0])  
            sphere_position_msg.y = float(rotated_position.flatten()[1])
            sphere_position_msg.z = float(rotated_position.flatten()[2])
                
            sphere.position = sphere_position_msg
            sphere.radius = r_sphere
            spheres.append(sphere)

        return spheres

# Create a Pose for testing
pose = Pose(0, 0, 0, 0, 0, 0, 1)  # Position at (0, 0, 0), no rotation (identity quaternion)

# Cuboid dimensions
x_len = 5
y_len = 5
z_len = 5

# Create a DroneSimulator and generate spheres
simulator = DroneSimulator()
spheres = simulator.cuboid_to_sphere(pose, x_len, y_len, z_len)

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract sphere positions
x = [sphere.position.x for sphere in spheres]
y = [sphere.position.y for sphere in spheres]
z = [sphere.position.z for sphere in spheres]

# Plot the spheres' centers
ax.scatter(x, y, z, c='r', marker='o', s=50, label="Sphere Centers")

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Cuboid Spheres')

plt.show()


'''
from casadi import MX, vertcat

# Define symbolic variables
x1 = MX.sym('x1')
x2 = MX.sym('x2')

# Stack variables into a vector
x = vertcat(x1, x2)
u = MX.sym('u')

# Define the system of equations (ODE)
ode = vertcat((1 - x2**2)*x1 - x2 + u, x1)

# Print the size of the ODE vector
print("Size of ode:", ode.size1(), ode.size2())
'''


'''
import pybullet as p
import pybullet_data

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create a plane and some obstacles
p.loadURDF("plane.urdf")
p.loadURDF("cube.urdf", [0, 0, 1])
p.loadURDF("cube.urdf", [1, 1, 1])

# Get all obstacles
num_bodies = p.getNumBodies()
obstacles = [p.getBodyUniqueId(i) for i in range(num_bodies)]

# Print details
print("All obstacles in the environment:", obstacles)
for body_id in obstacles:
    body_name = p.getBodyInfo(body_id)[1].decode('utf-8')
    print(f"Body ID: {body_id}, Name: {body_name}")

'''

'''
import numpy as np

def f(x, rng): 
    return rng.integers(1,10 )
#Intialise a random number generator
rng = np.random.default_rng(2021)
#pass the rng to functions which you would like to use it
random_number = f(1, rng)
print(random_number)

random_number = f(1, rng)
print(random_number)

random_number = f(1, rng)
print(random_number)

random_number = f(1, rng)
print(random_number)
'''

'''
import matplotlib.pyplot as plt
def calculate_column_positions(foundation_length, foundation_width):

    """
    Calculate the positions of columns on a foundation.

    Parameters:
        foundation_length (float): Length of the foundation in meters.
        foundation_width (float): Width of the foundation in meters.

    Returns:
        list: A list of (x, y) positions for the columns.
    """
   # Define parameters
    margin = 1  # Distance from the edges to the center of the columns (meters)

    # Compute the number of columns in each direction
    num_columns_x = int((foundation_length - 2 * margin) / 4) + 1
    num_columns_y = int((foundation_width - 2 * margin) / 4) + 1

    # Adjust spacing to ensure uniform distribution
    if num_columns_x > 1:
        spacing_x = (foundation_length - 2 * margin) / (num_columns_x - 1)
    else:
        spacing_x = 0

    if num_columns_y > 1:
        spacing_y = (foundation_width - 2 * margin) / (num_columns_y - 1)
    else:
        spacing_y = 0

    # Generate positions for columns
    positions = []
    for i in range(num_columns_x):
        for j in range(num_columns_y):
            x = margin + i * spacing_x
            y = margin + j * spacing_y
            positions.append((x, y))

    return positions

# Example usage:
foundation_length = 14  # Length of the foundation (meters)
foundation_width = 10   # Width of the foundation (meters)
column_positions = calculate_column_positions(foundation_length, foundation_width)

# Print results
print("Column positions:")
for pos in column_positions:
    print(pos)
    plt.scatter(pos[0],pos[1])
#plt.scatter(column_positions)
plt.show()

'''
