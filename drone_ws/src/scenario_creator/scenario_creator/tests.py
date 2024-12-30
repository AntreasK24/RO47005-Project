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