#sys.path.append("/gym-pybullet-drones")
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Adding ROS functionalities and drone interaction
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose,PoseArray
from std_msgs.msg import String
import drone_msgs.msg # import Obstacle, ObstacleArray, SphereArray
from gym_pybullet_drones.envs import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from std_msgs.msg import Float64MultiArray



class DroneSimulator(Node):  
    #Constructor
    def __init__(self):
        super().__init__('drone_simulator')
        
        #Subscriber
        self.velocity_subscriber = self.create_subscription(Twist,'/cmd_vel',self.velocity_callback,3)
        self.waypoint_subscriber = self.create_subscription(PoseArray,'/waypoints',self.waypoint_callback,1)
        self.sphere_subscriber = self.create_subscription(drone_msgs.msg.SphereArray,'/spheres',self.sphere_callback,1)
        self.target_position_sub = self.create_subscription(Float64MultiArray,'/target_pos',self.target_position_callback,10)

        #Publisher
        self.pose_publisher = self.create_publisher(Pose, '/pose', 10)
        self.obstacles_publisher = self.create_publisher(drone_msgs.msg.ObstacleArray, '/obstacles', 2)

        # Timer to publish drone poses and update obstacle lists
        self.timer_period = 1.0/240.0
        self.timer = self.create_timer(self.timer_period,self.timer_simulation)
        self.obstacle_timer = self.create_timer(0.1, self.timer_obstacles)
            # TODO: let static obstacles be handled by a service

        # Setting up the Enviroment
        self.velocity_subscriber ### Added from drone_simulator.py
        self.env = VelocityAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, ctrl_freq=240, gui=True)
        self.obs = self.env.reset()  


        #Variables for controlling drone 
        self.current_linear_velocity = np.array([[0.0, 0.0, 0.0, 1.0]])
        self.current_angular_velocity = np.array([[0.0, 0.0, 0.0]])
        self.current_velocity = np.hstack((self.current_linear_velocity,self.current_angular_velocity))

        self.drone_position = None 
        self.path = None #PathVisual Type
        self.spheres = None #SphereArray Type
        self.sphere_marker = None #Marker Type

        self.time = 0.0

        self.static_obstacles = []  # List to track static obstacles
        self.dynamic_obstacles = []  # List to track dynamic obstacles
        self.waypoints = [] # List of the current planned waypoints

        self.global_marker = []
        self.current_target = []
        self.targets = []

        # Create obstacles
        self.setup_environment()

    def setup_environment(self):
        
        # Mark start position
        start_marker = Marker(positions=[0.0,0.0,0.1125], radius=0.1, color=[0,1,0,0.3])
        self.global_marker.append(start_marker)

        ##### STATC OBSTACLES
            # Create a building (static obstacles)
        
        self.building = Building(
            storeys=2,
            position=[-2, -2, 0],
            #storey_kwargs={
            #    "column_kwargs": {"color": [1, 0, 0, 1], "radius": 0.3},
            #    "ceiling_kwargs": {"color": [0.2, 0.2, 0.8, 0.6]},
            #},
        )
        #building_info = self.building.create()
        #self.static_obstacles.extend(building_info) if self.static_obstacles is not None else self.static_obstacles.append(building_info) 
        

        cylinder = Obstacle(position=[1,1,1],length=2.0,radius=0.22,geom_shape='cylinder')
        cylinder_info = cylinder.create()
        self.static_obstacles.append(cylinder_info)

        # cylinder = Obstacle(position=[2,1,1],length=2.0,radius=0.1,geom_shape='cylinder')
        # cylinder_info = cylinder.create()
        # self.static_obstacles.append(cylinder_info)

        # cylinder = Obstacle(position=[3,1,1],length=2.0,radius=0.1,geom_shape='cylinder')
        # cylinder_info = cylinder.create()
        # self.static_obstacles.append(cylinder_info)



        # cylinder = Obstacle(position=[1,2,1],length=2.0,radius=0.1,geom_shape='cylinder')
        # cylinder_info = cylinder.create()
        # self.static_obstacles.append(cylinder_info)

        # cylinder = Obstacle(position=[1,3,1],length=2.0,radius=0.1,geom_shape='cylinder')
        # cylinder_info = cylinder.create()
        # self.static_obstacles.append(cylinder_info)

        '''
        ##### DYNAMIC OBSTACLES
            # Create a dynamic obstacle
        self.dynamic_obstacle = Obstacle(position=[0,3.0,1.0],length=2.0,radius=0.22,color=[0, 1, 0, 0.5],geom_shape='cylinder', dynamic=True)
        obstacle_info = self.dynamic_obstacle.create()
        self.static_obstacles.append(cylinder_info)
        self.dynamic_obstacles.append(obstacle_info) 
        '''


    def velocity_callback(self,msg):
        #Get linear and angular velocity and stack them into a single vector

        self.current_linear_velocity = np.array([[msg.linear.x,msg.linear.y,msg.linear.z,1]])
        self.current_angular_velocity = np.array([[msg.angular.x,msg.angular.y,msg.angular.z]])
        self.current_velocity = np.hstack((self.current_linear_velocity,self.current_angular_velocity))

        print(self.current_velocity)
    
    def waypoint_callback(self,msg):

        # Visualize a path - Example 
        #self.path_points = np.array([[0, 0, 0], [1, 1, 0.5], [2, 2, 1.5]])
        # msg is a List of 3D Points (geometry_msgs/msg/PoseArray - ignoring orientation)
        points = []
        points.append(self.drone_position)

        for pose in msg.poses:
            position = []
            position.append(pose.position.x)
            position.append(pose.position.y)
            position.append(pose.position.z)
            points.append(position)

        self.waypoints = points
        if self.path is not None:
            self.path.update(self.waypoints)
        else:
            self.path = PathVisual(self.waypoints)

    def target_position_callback(self,target_pos):
        x,y,z,vx,vy,vz = target_pos.data
        positions = []
        positions.append([x,y,z])
        self.current_target = positions[-1]
        for position in positions:        
            if position in self.global_marker:
                positions.remove(position)

        goal_marker = Marker(positions=positions, radius=0.1, color=[0,0,1,0.1])
        self.global_marker.append(goal_marker)
        self.targets.extend(positions)

    def sphere_callback(self,msg):
        self.spheres = msg.spheres # SphereArray

        if self.sphere_marker is not None:
            self.sphere_marker.update(sphere_array=self.spheres) #input: SphereArray
        else:
            self.sphere_marker = Marker(sphere_array=self.spheres) #input: SphereArray
        
    #def timer_static_obstacles(self):
    def timer_obstacles(self):
        # Publish static obstacles
        obstacles_msg = drone_msgs.msg.ObstacleArray()
        for obstacle in self.static_obstacles:
            static_obstacle = drone_msgs.msg.Obstacle()
            static_obstacle.shape = obstacle["geom_shape"]
            static_obstacle.pose = self.get_pose_from_obstacle(obstacle)
            if obstacle["geom_shape"] == "cylinder":
                static_obstacle.size = [obstacle["length"],obstacle["radius"]]
            else:    
                static_obstacle.size =  obstacle["size"]

            obstacles_msg.obstacles.append(static_obstacle)



        #def timer_dynamic_obstacles(self):
        if self.dynamic_obstacles:
            # Publish dynamic obstacles

            for obstacle in self.dynamic_obstacles:
                dynamic_obstacle = drone_msgs.msg.Obstacle()
                dynamic_obstacle.shape = obstacle["geom_shape"]
                dynamic_obstacle.pose = self.get_pose_from_obstacle(obstacle)
                if obstacle["geom_shape"] == "cylinder":
                    dynamic_obstacle.size = [obstacle["length"],obstacle["radius"]]
                else:    
                    dynamic_obstacle.size =  obstacle["size"]

                obstacles_msg.obstacles.append(dynamic_obstacle)

        self.obstacles_publisher.publish(obstacles_msg)
        
    def get_pose_from_obstacle(self, obstacle):
        # Convert obstacle position and orientation to Pose
        position, orientation = p.getBasePositionAndOrientation(obstacle["id"])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        return pose

    def timer_simulation(self):
        #p.stepSimulation()

        if self.dynamic_obstacles:
            ##### UPDATE obstacles
                # Calculate new position using a sine wave for smooth movement
            x_position = math.sin(self.time) * 2  # Oscillate between -2 and 2 along the x-axis
            self.dynamic_obstacle.update_pose(position=[x_position, 2.0, 0.5])
        

        #### Waypoint example
        #new_waypoints = self.path_points + np.array([[0,0,0],[0,0,x_position*0.1],[0,0,x_position*0.1]])
        #new_waypoints[0,:] = self.drone_position
        #self.path.update(new_waypoints)


        ##### UPDATE drone
            #Give velocity commands to drone and publish position
        pose_message = Pose()

        obs, reward, done, truncated, info = self.env.step(self.current_velocity)
        

        position = obs[0][0:3]
        self.drone_position = position
        pose_message.position.x = position[0]
        pose_message.position.y = position[1]
        pose_message.position.z = position[2]

        quaternion = obs[0][3:7]
        pose_message.orientation.x = quaternion[0]
        pose_message.orientation.y = quaternion[1]
        pose_message.orientation.z = quaternion[2]
        pose_message.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_message)

        ###### UPDATE waypoints
        if self.path:
            self.waypoints[0] = self.drone_position
            self.path.update(self.waypoints)

        # TODO self.dynamic_obstacles_publisher.publish()

        self.time += self.timer_period


class Obstacle():
    def __init__(self, **kwargs):
        # Default attributes
        defaults = {
            "id": None,
            "position": [0, 0, 0.5],
            "rotation": [0, 0, 0, 1], #No rotation (quarternion format)
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
        self.id = self.body
        return self.__dict__

    
    def update_pose(self,
                    position=None,
                    orientation=None):
        if self.dynamic:
                # Use current position/orientation if none provided
                position = position if position is not None else self.position
                orientation = orientation if orientation is not None else self.rotation
                
                # Update the Pose
                p.resetBasePositionAndOrientation(
                    self.body,
                    posObj=position,
                    ornObj=orientation,
                )
                self.position = position
                self.rotation = orientation
                return self.__dict__
        else:
            print("Trying to move a static object")

class Storey():
    def __init__(self, **kwargs):
        defaults = {
            "x_length": 6.0,
            "y_length": 8.0,
            "height": 4.0,
            "ceiling_thickness": 0.4, # TODO: Put it in ceiling_Kwargs
            "position": [0.0, 0.0, 0.0],
            "column_kwargs": {},  # Additional kwargs for columns
            "ceiling_kwargs": {}, # TODO: Put in thickness # Additional kwargs for ceiling
        }
        defaults.update(kwargs)
        self.__dict__.update(defaults)

        self.column_positions = self.calculate_column_positions(self.x_length, self.y_length)
        self.ceiling_thickness /= 2

        self.objects = []


    def calculate_column_positions(self, x_length, y_length):
        margin = 1  # Distance from edges to column center
        num_columns_x = int((x_length - 2 * margin) / 4) + 1 # 4 meters of spacing between columns
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
            column = Obstacle(
                position=[pos[0] + self.position[0], pos[1] + self.position[1], self.height / 2 + self.position[2]],
                length=self.height,
                geom_shape="cylinder",
                **self.column_kwargs,
            )
            col_info = column.create()
            self.objects.append(col_info)

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
        ceiling_info = ceiling.create()
        self.objects.append(ceiling_info)
        
        return self.objects

class Building():
    def __init__(self, **kwargs):
        defaults = {
            "storeys": 1,
            "position": [0.0, 0.0, 0.0],
            "storey_height": 3.0,
            "storey_ceiling_thickness": 0.4, # TODO: include in storey_kwargs
            "storey_kwargs": {},  # Additional kwargs for storeys
        }
        defaults.update(kwargs)
        self.__dict__.update(defaults)

        self.objects = []

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
            storey_info = storey.create()
            
            self.objects.extend(storey_info) if self.objects is not None else self.objects.append(storey_info)

        return self.objects

class Marker:
    def __init__(self, sphere_array= None, positions=None, radius=None, color=[1,0,0,0.3]):

        self.spheres = sphere_array # SphereArray
        self.positions = positions # list of lists [[x,y,z],...,[x,y,z]]
        self.radius = radius # float or int OR list of radii
        self.color = color # list [x,y,z,a]
        self._variable_radius = False
        self.point_ids = []  # Store visual body IDs for points

        # Convert positions to type list
        if self.positions is not None:
            # Convert positions to list if given in numpy
            self.positions = self._convert_to_list(self.positions)


        # Convert radii to type list
        if self.radius is not None:
            if not isinstance(self.radius,(int,float)):
                self._variable_radius = True
                assert (len(positions) == len(radius)) or isinstance(radius,(int,float) )
                self.radius = self._convert_to_list(self.radius)
            else:
                self._variable_radius = False

        self._create_visuals()  # Create the initial visuals

    def _convertSphereArray(self,sphere_array):
        if self.spheres is not None:
            self._variable_radius = True
            positions = []
            radii = []           
            for sphere in self.spheres:
                positions.append([sphere.position.x,sphere.position.y,sphere.position.z])
                radii.append(sphere.radius)
            self.positions = positions
            self.radius = radii

    def _convert_to_list(self, positions):
        """ Converts waypoints to a Python list if they are provided as an array """
        if isinstance(positions, np.ndarray):
            return positions.tolist()
        return positions

    def _create_visuals(self):
        """Creates points based on the positions."""
        if self.spheres is not None:
            self._convertSphereArray(self.spheres)
            self._variable_radius = True        
        
        self._clear_visuals()
        
        if self._variable_radius:
        # Create points as small spheres
            for i in range(len(self.positions)):
                visual_shape_id = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=self.radius[i],
                    rgbaColor=self.color,
                )
                point_id = p.createMultiBody(
                    baseMass=0,  # No dynamics
                    baseVisualShapeIndex=visual_shape_id,
                    basePosition=self.positions[i],
                )
                self.point_ids.append(point_id)
        else:
            print('HELP')
            for point in self.positions:
                visual_shape_id = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=self.radius,
                    rgbaColor=self.color,
                )
                point_id = p.createMultiBody(
                    baseMass=0,  # No dynamics
                    baseVisualShapeIndex=visual_shape_id,
                    basePosition=point,
                )
                self.point_ids.append(point_id)

    def update(self, sphere_array, positions=None, radius=None):
        """ Updates the waypoint positions to safe computational power. Varying lengths of waypoints should be handled """

        self._convertSphereArray(sphere_array)
        self.spheres = sphere_array
        self._variable_radius = True

        if positions and self.radius:
            positions = self._convert_to_list(positions)
            self.positions = positions
            if not isinstance(self.radius,(int,float)):
                self._variable_radius = True
                assert (len(positions) == len(radius)) or isinstance(radius,(int,float) )
                # Check if we have the same number of positions as of radii
                self.radius = self._convert_to_list(self.radius)
            else:
                self._variable_radius = False
                self.radius = radius
        
        ##### RADII CANNOT BE DYNAMICALLY ADJUSTED, THE OBJECT HAS TO BE REMOVED

        # Handle varying lengths: Create or remove spheres as necessary
        # Step 1: Create additional points in the origin
        
        if not self._variable_radius:

            while len(self.point_ids) < len(self.positions):
                visual_shape_id = p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=self.radius,
                    rgbaColor=self.color,
                )
                point_id = p.createMultiBody(
                    baseMass=0,  # No dynamics
                    baseVisualShapeIndex=visual_shape_id,
                    basePosition=[0, 0, 0],  # Initial position (will be updated)
                )
                self.point_ids.append(point_id)

            # Step 2: Remove the last points if we have too many
            while len(self.point_ids) > len(self.positions):
                p.removeBody(self.point_ids.pop())
            
            # Step3: Update points
            # Update positions of points
            for i, pos in enumerate(self.positions):
                p.resetBasePositionAndOrientation(self.point_ids[i], pos, [0, 0, 0, 1])
            
        
        else:
            assert len(self.positions) == len(self.radius)
            if len(self.positions) == len(self.radius):
                while len(self.point_ids) < len(self.positions):
                    visual_shape_id = p.createVisualShape(
                        shapeType=p.GEOM_SPHERE,
                        radius=self.radius[0], # take the first radius
                        rgbaColor=self.color,
                    )
                    point_id = p.createMultiBody(
                        baseMass=0,  # No dynamics
                        baseVisualShapeIndex=visual_shape_id,
                        basePosition=[0, 0, 0],  # Initial position (will be updated)
                    )
                    self.point_ids.append(point_id)

                # Step 2: Remove the last points if we have too many
                while len(self.point_ids) > len(self.positions):
                    p.removeBody(self.point_ids.pop())
                
                # Step3: Update points
                # Update positions of points
                changed_radii = []
                for i, pos in enumerate(self.positions):
                    if p.getVisualShapeData(self.point_ids[i])[0][3][0] == self.radius[i]:
                        p.resetBasePositionAndOrientation(self.point_ids[i], pos, [0, 0, 0, 1])
                    else:
                        changed_radii.append([self.point_ids[i],i])

                # Remove IDs with changed radii and create them new:#
                if changed_radii:
                    changed_spheres = 0
                    changed_radii = changed_radii.sort()
                    for id, i in changed_radii:
                        i -= changed_spheres
                        p.removeBody(id) # delete body
                        # delete entry in positions list
                        
                        visual_shape_id = p.createVisualShape(
                            shapeType=p.GEOM_SPHERE,
                            radius=self.radius[i], # take the first radius
                            rgbaColor=self.color,
                        )
                        point_id = p.createMultiBody(
                            baseMass=0,  # No dynamics
                            baseVisualShapeIndex=visual_shape_id,
                            basePosition=self.positions[i],  # Initial position (will be updated)
                        )
                        self.point_ids.append(point_id)
                        # move the position and radius of the changed spheres to the end of the 
                        self.positions.append(self.positions.pop(i))
                        self.radius.append(self.radius.pop(i))

                        changed_spheres += 1

            else:
                print("Give the same langth of positions and radii")


    def _clear_visuals(self):
        """Removes all debug lines and points"""
        for point_id in self.point_ids:
            p.removeBody(point_id)
        
        self.point_ids.clear()

    def __del__(self):
        """Ensures that all visuals are removed when the object is deleted"""
        try:
            self._clear_visuals()
        except Exception:
            pass

class PathVisual:
    def __init__(self, waypoints=None, line_color=[0, 1, 0], point_color=[1, 0, 0, 1], line_width=2, point_radius=0.02):
        """
        This class is able to visualize planned waypoints in the simulation environment
        
        Parameters:
            waypoints (list or np.ndarray): List or NumPy array of 3D waypoints as [[x1, y1, z1], [x2, y2, z2], ...].
        """
        self.waypoints = self._convert_to_list(waypoints) if waypoints else []
        self.line_color = line_color
        self.point_color = point_color
        self.line_width = line_width
        self.point_radius = point_radius
        
        self.line_ids = []  # Store IDs of debug lines
        self.point_ids = []  # Store visual body IDs for points
        
        self._create_visuals()  # Create the initial visuals
    
    def _convert_to_list(self, waypoints):
        """ Converts waypoints to a Python list if they are provided as a NumPy array """
        if isinstance(waypoints, np.ndarray):
            return waypoints.tolist()
        return waypoints

    def _create_visuals(self):
        """Creates the initial debug lines and points based on the waypoints."""
        self._clear_visuals()
        
        # Create points as small spheres
        for wp in self.waypoints:
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=self.point_radius,
                rgbaColor=self.point_color,
            )
            point_id = p.createMultiBody(
                baseMass=0,  # No dynamics
                baseVisualShapeIndex=visual_shape_id,
                basePosition=wp,
            )
            self.point_ids.append(point_id)
        
        # Create lines between consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            line_id = p.addUserDebugLine(
                lineFromXYZ=self.waypoints[i],
                lineToXYZ=self.waypoints[i + 1],
                lineColorRGB=self.line_color,
                lineWidth=self.line_width,
            )
            self.line_ids.append(line_id)
     
    def update(self, waypoints):
        """ Updates the waypoint positions to safe computational power. Varying lengths of waypoints should be handled """
        waypoints = self._convert_to_list(waypoints)

        # Handle varying lengths: Create or remove spheres and lines as necessary
        # Step 1: Create additional points and lines in the origin
        while len(self.point_ids) < len(waypoints):
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=self.point_radius,
                rgbaColor=self.point_color,
            )
            point_id = p.createMultiBody(
                baseMass=0,  # No dynamics
                baseVisualShapeIndex=visual_shape_id,
                basePosition=[0, 0, 0],  # Initial position (will be updated)
            )
            self.point_ids.append(point_id)

        while len(self.line_ids) < len(waypoints) - 1:
            line_id = p.addUserDebugLine(
                lineFromXYZ=[0, 0, 0],  # Initial position (will be updated)
                lineToXYZ=[0, 0, 0],
                lineColorRGB=self.line_color,
                lineWidth=self.line_width,
            )
            self.line_ids.append(line_id)



        # Step 2: Remove the last points and lines of we have too many
        while len(self.point_ids) > len(waypoints):
            p.removeBody(self.point_ids.pop())
        
        while len(self.line_ids) > len(waypoints) - 1:
            p.removeUserDebugItem(self.line_ids.pop())
        
        # Step3: Update points and lines
        # Update positions of points
        for i, wp in enumerate(waypoints):
            p.resetBasePositionAndOrientation(self.point_ids[i], wp, [0, 0, 0, 1])
        
        # Update positions of lines
        for i in range(len(waypoints) - 1):
            p.addUserDebugLine(
                lineFromXYZ=waypoints[i],
                lineToXYZ=waypoints[i + 1],
                lineColorRGB=self.line_color,
                lineWidth=self.line_width,
                replaceItemUniqueId=self.line_ids[i],
            )

        self.waypoints = waypoints

    def _clear_visuals(self):
        """Removes all debug lines and points"""
        for line_id in self.line_ids:
            p.removeUserDebugItem(line_id)
        for point_id in self.point_ids:
            p.removeBody(point_id)
        
        self.line_ids.clear()
        self.point_ids.clear()

    def __del__(self):
        """Ensures that all visuals are removed when the object is deleted"""
        try:
            self._clear_visuals()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    drone_simulator = DroneSimulator()
    rclpy.spin(drone_simulator)
    drone_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# TODO: ROS params ,
# TODO: randomization with random seed, 



# TODO: Implement different difficulties

# TODO: input validation
    # if not isinstance(value, int):
    #    raise TypeError("Value must be an integer")
    # if value <= 0:
    #    raise ValueError("Value must be greater than zero")

# TODO: Change static obstacle publisher to a service


''' OLD non-ROS main loop: 
def main(args=None):
        # Connect to PyBullet simulation
    physics_client = p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set path to PyBullet data

    p.setGravity(0, 0, -9.81)  # Set gravity
    plane_id = p.loadURDF("plane.urdf")  # Load a plane

    time_step = 1 / 240  # Simulate manually with real-time pacing
    #p.setRealTimeSimulation(1) (does not work but should in theory)
    t = 0  # Time variable for dynamic obstacles


    building = Building(
        storeys=3,
        position=[-2, -2, 0],
        #storey_kwargs={
        #    "column_kwargs": {"color": [1, 0, 0, 1], "radius": 0.3},
        #    "ceiling_kwargs": {"color": [0.2, 0.2, 0.8, 0.6]},
        #},
    )
    building.create()


    dynamic_obstacle = Obstacle(color=[0,1,0,0.5], dynamic=True)
    dynamic_obstacle.create()


    # Visualize a path 
    path_points = np.array([[0, 0, 0], [1, 1, 0.5], [2, 2, 1.5]])
    print(path_points)
    path = PathVisual(path_points)
    print(path.waypoints)


    while p.isConnected():
        p.stepSimulation()
        time.sleep(time_step)  # Pause to match real time

        
        # Calculate new position using a sine wave for smooth movement
        x_position = math.sin(t) * 2  # Oscillate between -2 and 2 along the x-axis
        dynamic_obstacle.update_pose(position=[x_position, 0, 0.5],orientation=[0, 0, 0, 1])

        new_waypoints = path_points + np.array([[0,0,0],[0,0,x_position*0.1],[0,0,x_position*0.1]])
        path.update(new_waypoints)

        t += time_step  # Increment time
        

    p.disconnect()
'''