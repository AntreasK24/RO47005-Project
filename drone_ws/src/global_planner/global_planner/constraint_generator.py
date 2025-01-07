import casadi as ca
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class ConstraintNode(Node):
    def __init__(self):
        super().__init__('constraint_node')
        self.subscription = self.create_subscription(
            Point,
            'drone_position',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'cylinder_constraints',
            10)

    def generate_cylinder_constraints(self, x, y, cylinder_centers, cylinder_radii):
        constraints = []
        for (cx, cy), r in zip(cylinder_centers, cylinder_radii):
            dx = x - cx
            dy = y - cy
            dist = ca.sqrt(dx**2 + dy**2)
            normal_x = dx / (dist + 1e-6)
            normal_y = dy / (dist + 1e-6)
            constraint = normal_x * (x - cx) + normal_y * (y - cy) - r
            constraints.append(float(constraint))
        return constraints

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        cylinder_centers = [(5, 5), (10, 10)]
        cylinder_radii = [2, 3]
        constraints = self.generate_cylinder_constraints(x, y, cylinder_centers, cylinder_radii)
        msg = Float32MultiArray()
        msg.data = constraints
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConstraintNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
