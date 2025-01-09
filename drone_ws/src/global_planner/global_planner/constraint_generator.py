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
        # we will 
        orientation = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        rotation_matrix = self.quaternion_to_rotation_matrix(orientation)

        cylinder_axis = rotation_matrix[:, 2:3]
        base_position = np.array([[pose.position.x],[pose.position.y],[pose.position.z]])
        base_position = base_position - cylinder_axis * length / 2

        r_sphere = np.sqrt(r_cyl**2 + r_cyl**2) # sphere radius - will be 1.41 * radius_cylinder

        num_spheres = int(length // r_cyl)

        for i in range(num_spheres):
            sphere = drone_msgs.msg.Sphere()
            sphere_position = base_position + cylinder_axis * (i+1) * r_sphere

            sphere.position.x = sphere_position[0,0]
            sphere.position.y = sphere_position[1,0]
            sphere.position.z = sphere_position[2,0]
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
                spheres = self.cylinder_to_sphere(pose, length, radius, 'cylinder')
                sphere_array.extend(spheres)
            elif obstacle.shape == "cubiod":
                
                # TODO: implementation
                
                pass
        
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
