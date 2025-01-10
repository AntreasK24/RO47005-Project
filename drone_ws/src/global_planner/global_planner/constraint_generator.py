import rclpy
import numpy as np
from rclpy.node import Node
import drone_msgs.msg # import Obstacle, ObstacleArray
from geometry_msgs.msg import Point

class ConstraintNode(Node):
    def __init__(self):
        super().__init__('constraint_node')
        self.static_subscription = self.create_subscription(
            drone_msgs.msg.ObstacleArray,
            '/static_obstacles',
            self.static_obstacles_callback,
            3)
        
        '''
        self.dynamic_subscription = self.create_subscription(
            drone_msgs.msg.ObstacleArray,
            '/dynamic_obstacles',
            self.dynamic_obstacles_callback,
            3)
        '''
            
        self.publisher = self.create_publisher(
            drone_msgs.msg.SphereArray,
            '/spheres',
            3)
        
    def quaternion_to_rotation_matrix(self,q):
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2,   2*x*y - 2*z*w,          2*x*z + 2*y*w],
            [2*x*y + 2*z*w,         1 - 2*x**2 - 2*z**2,    2*y*z - 2*x*w],
            [2*x*z - 2*y*w,         2*y*z + 2*x*w,          1 - 2*x**2 - 2*y**2]
        ])


    def cylinder_to_sphere(self, pose, length, r_cyl, shape):
        spheres = []
        # calculate the desired positions and radius of the spheres
        
        orientation = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation)

        cylinder_axis = rotation_matrix[:, 2:3]
        base_position = np.array([[pose.position.x],[pose.position.y],[pose.position.z]])
        base_position = base_position - cylinder_axis * length / 2

        r_sphere = np.sqrt(r_cyl**2 + r_cyl**2) # sphere radius - will be 1.41 * radius_cylinder
        
        #self.get_logger().info(f'Base position: {base_position}', once=True)
        
        num_spheres = int(length // r_cyl)
        #self.get_logger().info(f'Number of spheres: {num_spheres}', once=True)

        for i in range(1,num_spheres):
            sphere = drone_msgs.msg.Sphere()
            sphere_position = base_position + cylinder_axis * i * r_cyl

            sphere.position.x = sphere_position[0,0]
            sphere.position.y = sphere_position[1,0]
            sphere.position.z = sphere_position[2,0]
            sphere.radius = r_sphere
            spheres.append(sphere)
            
            # Check if enough spheres are created or if another has to be put at the end
            if i == num_spheres - 1:
                dist_lastSphere_end = np.linalg.norm((base_position + cylinder_axis * length)-(base_position + cylinder_axis * i * r_cyl))
                self.get_logger().info(f'Residual distance: {dist_lastSphere_end}', once=True)
                if dist_lastSphere_end > r_cyl:
                    sphere = drone_msgs.msg.Sphere()
                    sphere_position = base_position + cylinder_axis * length - cylinder_axis * r_cyl

                    sphere.position.x = sphere_position[0,0]
                    sphere.position.y = sphere_position[1,0]
                    sphere.position.z = sphere_position[2,0]
                    sphere.radius = r_sphere
                    spheres.append(sphere)

        return spheres

    def cuboid_to_sphere(self, pose,x_len,y_len,z_len):
        spheres = []
        # calculate the desired positions and radius of the spheres
        # get the position
        center_position = np.array([[pose.position.x],[pose.position.y],[pose.position.z]])

        # get the orientation
        orientation = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation)

        base_position = center_position - rotation_matrix.dot(np.array([[x_len], [y_len], [z_len]]))

        # get the size
        size = [x_len,y_len,z_len]
        r_obj = 1.0 if min(size) >= 1.0 else (0.3 if min(size) <= 0.3 else min(size))
        r_sphere = np.sqrt(r_obj**2 + r_obj**2) # sphere radius - will be 1.41 * radius_cylinder

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
            sphere = drone_msgs.msg.Sphere()
            sphere_position = np.array([Xs[i], Ys[i], Zs[i]])
            
            # Apply the rotation to the spheres
            rotated_position = rotation_matrix.dot(sphere_position - center_position)
            rotated_position = rotated_position + center_position

            # Set sphere position and radius
            sphere.position.x = float(rotated_position.flatten()[0])  
            sphere.position.y = float(rotated_position.flatten()[1])
            sphere.position.z = float(rotated_position.flatten()[2])
            sphere.radius = r_sphere
            spheres.append(sphere)

        return spheres


    def obstacles_to_spheres(self, obstacle_array):
        sphere_array = []
        obstacles = obstacle_array.obstacles

        for obstacle in obstacles:
            if obstacle.shape == "cylinder":
                pose = obstacle.pose #Position is a geometry_msgs/msg/Point
                length, radius = obstacle.size
                spheres = self.cylinder_to_sphere(pose, length, radius)
                sphere_array.extend(spheres)
            elif obstacle.shape == "cubiod":
                pose = obstacle.pose
                x_len, y_len, z_len = obstacle.size
                spheres = self.cuboid_to_sphere(pose,x_len,y_len,z_len)
                sphere_array.extend(spheres)
        
        return sphere_array

    def dynamic_obstacles_callback(self, dynamic_obstacle_array):
        pub_msg = drone_msgs.msg.SphereArray()
        pub_msg.spheres = self.obstacles_to_spheres(dynamic_obstacle_array)
        self.publisher.publish(pub_msg)


    def static_obstacles_callback(self, static_obstacle_array):
        pub_msg = drone_msgs.msg.SphereArray()
        pub_msg.spheres = self.obstacles_to_spheres(static_obstacle_array)
        self.publisher.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConstraintNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
