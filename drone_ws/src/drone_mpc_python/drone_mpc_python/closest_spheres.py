import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from drone_msgs.msg import SphereArray
from datetime import datetime
import json  # To handle saving logs as JSON

class ClosestSphereDistance(Node):
    def __init__(self):
        super().__init__('closest_sphere_distance')
        
        # Initialize the drone's position
        self.drone_position = Point()
        
        # List to store closest distances
        self.closest_distances = []

        # Timer for rate-limiting the processing
        self.timer_period = 1.0  # Timer interval in seconds (1 Hz)
        self.timer = self.create_timer(self.timer_period, self.process_data)

        # Placeholder for sphere data
        self.sphere_data = None
        
        # Subscribe to the /pose topic to get the drone's pose
        self.pose_subscription = self.create_subscription(
            Pose,          # Message type for the topic
            '/pose',       # Topic name
            self.pose_callback,
            10
        )
        
        # Subscribe to the /spheres topic to get the spheres' data
        self.spheres_subscription = self.create_subscription(
            SphereArray,   # Message type for the topic
            '/spheres',    # Topic name
            self.sphere_callback,
            10
        )

        self.logs = {"closest_distances": []}

    def save_logs(self):
        current_time = datetime.now()
        unique_name = current_time.strftime("%Y%m%d_%H%M%S")
        filename = "logs_closest_spheres_" + unique_name + ".json"

        # Save the logs to a file
        with open(filename, 'w') as file:
            json.dump(self.logs, file, indent=4)

        self.get_logger().info(f"Logs saved to {filename}")

    def pose_callback(self, msg):
        # Update the drone's position from the /pose topic
        self.drone_position.x = msg.position.x
        self.drone_position.y = msg.position.y
        self.drone_position.z = msg.position.z
        #self.get_logger().info(f'Updated drone position: ({self.drone_position.x}, {self.drone_position.y}, {self.drone_position.z})')

    def sphere_callback(self, msg):
        # Update the sphere data
        self.sphere_data = msg

    def process_data(self):
        # Process data at a controlled rate
        if self.sphere_data is None:
            self.get_logger().warn('No sphere data available yet.')
            return

        closest_distance = float('inf')  # Start with a large value

        # Iterate through each sphere in the SphereArray message
        for sphere in self.sphere_data.spheres:
            sphere_center = sphere.position  # Center of the sphere
            sphere_radius = sphere.radius   # Radius of the sphere

            # Calculate Euclidean distance between drone and sphere center
            distance_to_center = ((self.drone_position.x - sphere_center.x) ** 2 +
                                  (self.drone_position.y - sphere_center.y) ** 2 +
                                  (self.drone_position.z - sphere_center.z) ** 2) ** 0.5

            # Calculate the distance to the surface of the sphere
            distance_to_surface = max(0.0, distance_to_center - sphere_radius)

            # Update closest distance
            if distance_to_surface < closest_distance:
                closest_distance = distance_to_surface

        # Append the closest distance to the list
        self.closest_distances.append(closest_distance)
        self.logs["closest_distances"].append(closest_distance)
        
        # Log the closest distance and the list
        self.get_logger().info(f'Closest distance to a sphere: {closest_distance}')
        self.get_logger().info(f'All closest distances so far (length={len(self.closest_distances)}): {self.closest_distances}')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ClosestSphereDistance()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Saving logs...")
        node.save_logs()  # Save logs when program is interrupted

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
