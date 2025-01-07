import pybullet as p
import pybullet_data
import time
import math


'''
import numpy as np
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Pose
from gym_pybullet_drones.envs import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
'''

# Connect to PyBullet simulation
physics_client = p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set path to PyBullet data

p.setGravity(0, 0, -9.81)  # Set gravity
plane_id = p.loadURDF("plane.urdf")  # Load a plane

'''

class Obstacle():
    def __init__(self,id=None, 
                position=[0, 0, 0.5],
                rotation=None,
                linear_velocity=None,
                angular_velocity=None,

                size=[0.5, 0.5, 0.5],       # Half-size-extend in xyz for cuboids
                radius=0.5,                 # radius in m
                length=3.0,                 # size in m
                geom_shape="cuboid",

                dynamic=False,              # static = False | dynamic = True 
                mass=0,                     # mass in kg, 0 means the object in static

                color=[0.5, 0.5, 0.5, 1.0],       #RGBA
                ):
        
        
        print(geom_shape)
        self.id = id                # To identify the object
        # POSE
        self.position = position
        self.rotation = rotation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        
        # GEOMETRY
        self.size = size
        self.radius = radius
        self.length = length
        self.geom_shape = geom_shape #cuboid or cylinder

        # DYNAMICS
        self.dynamic = dynamic # static = False | dynamic = True 
        self.mass = mass


        # VISUAL
        self.color = color # RGBA-list
        self.collision = None
        self.visual = None
        self.body = None

        if self.geom_shape != "cuboid" and self.geom_shape != "cylinder":
            self.geom_shape = "cuboid"
            print("The obstacles has no valid shape type. Use 'cuboid' or'cylinder'.")

        #self.create()

    def create(self):
        if self.geom_shape == "cuboid":
            # Create a cuboid
            self.collision = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.size  # half-lengths of the cuboid
            )
            self.visual = p.createVisualShape(
                p.GEOM_BOX, halfExtents=self.size, rgbaColor=self.color
            )
            self.body = p.createMultiBody(
                baseMass=self.mass,
                baseCollisionShapeIndex=self.collision,
                baseVisualShapeIndex=self.visual,
                basePosition=self.position  # position of the cuboid
            )

        elif self.geom_shape == "cylinder":
            # Create a cylinder
            self.collision = p.createCollisionShape(
                p.GEOM_CYLINDER, radius=self.radius, height=self.length
            )
            self.visual = p.createVisualShape(
                p.GEOM_CYLINDER, radius=self.radius, length=self.length, rgbaColor=self.color
            )
            self.body = p.createMultiBody(
                baseMass=self.mass,
                baseCollisionShapeIndex=self.collision,
                baseVisualShapeIndex=self.visual,
                basePosition=self.position  # position of the cylinder
            )
        else:
            print("No valid obstacle type given")


class Storey():
    def __init__(self,
                 x_length=10.0,
                 y_length=20.0,
                 height=3.0,
                 ceiling_thickness=0.4,
                 position=[0.0,0.0,0.0]):
        self.x_length = x_length  # Length of the foundation (meters)
        self.y_length = y_length   # Width of the foundation (meters)
        self.height = height
        self.position = position
        self.column_positions = self.calculate_column_positions(self.x_length, self.y_length)
        self.ceiling_thickness = ceiling_thickness/2
        print('INFO: initialzie storey')

    def calculate_column_positions(self,x_length, y_length):

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
        num_columns_x = int((x_length - 2 * margin) / 4) + 1
        num_columns_y = int((y_length - 2 * margin) / 4) + 1

        # Adjust spacing to ensure uniform distribution
        if num_columns_x > 1:
            spacing_x = (x_length - 2 * margin) / (num_columns_x - 1)
        else:
            spacing_x = 0

        if num_columns_y > 1:
            spacing_y = (y_length - 2 * margin) / (num_columns_y - 1)
        else:
            spacing_y = 0

        # Generate positions for columns
        positions = []
        for i in range(num_columns_x):
            for j in range(num_columns_y):
                x = margin + i * spacing_x
                y = margin + j * spacing_y
                positions.append((x, y))
        print('INFO: collum positions calculated')
        return positions

    def create(self):
        for pos in self.column_positions:
            obstacle = Obstacle(geom_shape='cylinder',
                                length=self.height,
                                position=[pos[0]+self.position[0],
                                          pos[1]+self.position[1],
                                          self.height/2+self.position[2]])
            obstacle.create()
            print('INFO: collum created')
        

        ceiling = Obstacle(geom_shape='cuboid',
                           position=[self.x_length/2+self.position[0],
                                     self.y_length/2+self.position[1],
                                     (self.height+self.ceiling_thickness+self.position[2])],
                           size=[self.x_length/2,self.y_length/2,self.ceiling_thickness],
                           color=[0.5,0.5,0.5,0.8]
                           )
        ceiling.create()
        print('INFO: storey created')



class Building():
    def __init__(self,storeys=1,position=[0.0,0.0,0.0],storey_height=3.0,storey_ceiling_thickness=0.4):
        self.storeys = int(storeys)
        self.position = position
        self.storey_ceiling_thickness = storey_ceiling_thickness
        self.storey_height = storey_height
        self.height = self.storey_height + self.storey_ceiling_thickness

    def create(self):
        for storey in range(self.storeys):
            Storey(position=[self.position[0],self.position[1],self.position[2]+storey*self.height]).create()


dynamic_obstacle = Obstacle(dynamic=True)
dynamic_obstacle.create()
#Set up obstacles:

'''



class Obstacle():
    def __init__(self, **kwargs):
        # Default attributes
        defaults = {
            "id": None,
            "position": [0, 0, 0.5],
            "rotation": None,
            "linear_velocity": None,
            "angular_velocity": None,
            "size": [0.5, 0.5, 0.5],  # Half-size-extent in xyz for cuboids
            "radius": 0.5,  # Radius in meters
            "length": 3.0,  # Size in meters
            "geom_shape": "cuboid",
            "dynamic": False,  # Static = False, dynamic = True
            "mass": 0,  # Mass in kg, 0 means the object is static
            "color": [0.5, 0.5, 0.5, 1.0],  # RGBA
        }
        # Update with provided values
        defaults.update(kwargs)
        self.__dict__.update(defaults)

        # Validation for geometry type
        if self.geom_shape not in ["cuboid", "cylinder"]:
            self.geom_shape = "cuboid"
            print("Invalid geom_shape; defaulting to 'cuboid'.")
        


    def create(self):
        if self.geom_shape == "cuboid":
            self.collision = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=self.size
            )
            self.visual = p.createVisualShape(
                p.GEOM_BOX, halfExtents=self.size, rgbaColor=self.color
            )
        elif self.geom_shape == "cylinder":
            self.collision = p.createCollisionShape(
                p.GEOM_CYLINDER, radius=self.radius, height=self.length
            )
            self.visual = p.createVisualShape(
                p.GEOM_CYLINDER, radius=self.radius, length=self.length, rgbaColor=self.color
            )
        self.body = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=self.collision,
            baseVisualShapeIndex=self.visual,
            basePosition=self.position,
        )
        print('Visual id:')
        print(self.visual)
        print('Collision id')
        print(self.collision)
        print('Body ID')
        print(self.body)

class Storey():
    def __init__(self, **kwargs):
        defaults = {
            "x_length": 10.0,
            "y_length": 20.0,
            "height": 3.0,
            "ceiling_thickness": 0.4,
            "position": [0.0, 0.0, 0.0],
            "column_kwargs": {},  # Additional kwargs for columns
            "ceiling_kwargs": {},  # Additional kwargs for ceiling
        }
        defaults.update(kwargs)
        self.__dict__.update(defaults)

        self.column_positions = self.calculate_column_positions(self.x_length, self.y_length)
        self.ceiling_thickness /= 2

    def calculate_column_positions(self, x_length, y_length):
        margin = 1  # Distance from edges to column center
        num_columns_x = int((x_length - 2 * margin) / 4) + 1
        num_columns_y = int((y_length - 2 * margin) / 4) + 1
        spacing_x = (x_length - 2 * margin) / max(1, (num_columns_x - 1))
        spacing_y = (y_length - 2 * margin) / max(1, (num_columns_y - 1))
        positions = [
            (margin + i * spacing_x, margin + j * spacing_y)
            for i in range(num_columns_x)
            for j in range(num_columns_y)
        ]
        return positions

    def create(self):
        # Create columns
        for pos in self.column_positions:
            obstacle = Obstacle(
                position=[pos[0] + self.position[0], pos[1] + self.position[1], self.height / 2 + self.position[2]],
                length=self.height,
                geom_shape="cylinder",
                **self.column_kwargs,
            )
            obstacle.create()

        # Create ceiling
        ceiling = Obstacle(
            position=[
                self.x_length / 2 + self.position[0],
                self.y_length / 2 + self.position[1],
                self.height + self.ceiling_thickness + self.position[2],
            ],
            size=[self.x_length / 2, self.y_length / 2, self.ceiling_thickness],
            geom_shape="cuboid",
            **self.ceiling_kwargs,
        )
        ceiling.create()

class Building():
    def __init__(self, **kwargs):
        defaults = {
            "storeys": 1,
            "position": [0.0, 0.0, 0.0],
            "storey_height": 3.0,
            "storey_ceiling_thickness": 0.4,
            "storey_kwargs": {},  # Additional kwargs for storeys
        }
        defaults.update(kwargs)
        self.__dict__.update(defaults)


    def create(self):
        for i in range(self.storeys):
            storey_position = [
                self.position[0],
                self.position[1],
                self.position[2] + i * (self.storey_height + self.storey_ceiling_thickness),
            ]
            storey = Storey(
                position=storey_position,
                height=self.storey_height,
                ceiling_thickness=self.storey_ceiling_thickness,
                **self.storey_kwargs,
            )
            storey.create()


building = Building(
    storeys=3,
    position=[-2, -2, 0],
    #storey_kwargs={
    #    "column_kwargs": {"color": [1, 0, 0, 1], "radius": 0.3},
    #    "ceiling_kwargs": {"color": [0.2, 0.2, 0.8, 0.6]},
    #},
)
building.create()
print('HELLLOOOOO')
print(building)

dynamic_obstacle = Obstacle(color=[0,1,0,0.5], mass=1)
dynamic_obstacle.create()


def create_marker_sphere(position, radius=0.1, color=[1, 1, 0, 1]):
    """Creates a visual marker sphere at a given position."""
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color,
    )
    p.createMultiBody(
        baseMass=0,  # No dynamics
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position,
    )

def draw_debug_line(start, end, color=[0, 1, 0], width=2):
    """Draws a debug line between two points."""
    p.addUserDebugLine(
        lineFromXYZ=start,
        lineToXYZ=end,
        lineColorRGB=color[:3],  # Only RGB is used for lines
        lineWidth=width,
    )

# Visualize a path with spheres and lines
path_points = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]  # Example points
for point in path_points:
    create_marker_sphere(position=point, radius=0.1, color=[1, 0, 0, 1])  # Red spheres

for i in range(len(path_points) - 1):
    draw_debug_line(
        start=path_points[i], 
        end=path_points[i + 1], 
        color=[0, 0, 1],  # Blue lines
        width=2,
    )





# Run the simulation until the window is closed
# Simulate manually with real-time pacing
time_step = 1 / 240  # Default PyBullet time step

#p.setRealTimeSimulation(1) (funktioniert nicht)
# Move the obstacle back and forth
t = 0  # Time variable for motion

while p.isConnected():
    p.stepSimulation()
    time.sleep(time_step)  # Pause to match real time

    
     # Calculate new position using a sine wave for smooth movement
    x_position = math.sin(t) * 2  # Oscillate between -2 and 2 along the x-axis
    p.resetBasePositionAndOrientation(
        dynamic_obstacle.body,
        posObj=[x_position, 0, 0.5],  # Update position
        ornObj=[0, 0, 0, 1]  # No rotation (quaternion format)
    )

    t += time_step  # Increment time
    

p.disconnect()


'''

def main(args=None):
    rclpy.init(args=args)
    drone_simulator = DroneSimulator()
    rclpy.spin(drone_simulator)
    drone_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''



# TODO: Make all the parameters from the lowlevel-class "Obstacle" available when calling "Building"

# TODO: ROS params ,
# TODO: randomization with random seed, 
# TODO: dynamic, 
# TODO: publish static obstacles ones, 
# TODO: dynamic obstacles
# TODO: publish dynamic obstacles,
# TODO: publish types of obstacles 
# TODO: input validation
    # if not isinstance(value, int):
    #    raise TypeError("Value must be an integer")
    # if value <= 0:
    #    raise ValueError("Value must be greater than zero")


### ROS MESSAGE shape_msgs/msg/SolidPrimitive Message https://docs.ros2.org/foxy/api/shape_msgs/msg/SolidPrimitive.html

