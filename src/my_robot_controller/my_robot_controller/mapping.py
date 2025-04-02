import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(LaserScan, "/scan", self.robot_controller, 10)

        # Movement parameters
        self.obstacle_threshold = 1.0  # Minimum safe distance (meters)
        self.angular_velocity = 0.5  # Turn speed
        self.linear_velocity = 0.3  # Forward speed
        self.last_turn_direction = "right"  # Keep track of last turn to alternate

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Extract directional distances from the laser scan data
        front = min(scan.ranges[:15] + scan.ranges[-15:])  # Front (0-30 degrees)
        left = min(scan.ranges[60:120])  # Left side (60-120 degrees)
        right = min(scan.ranges[240:300])  # Right side (240-300 degrees)

        self.get_logger().info(f"Front: {front}, Left: {left}, Right: {right}")

        # If an obstacle is ahead, decide to turn
        if front < self.obstacle_threshold:
            self.get_logger().info("Obstacle detected ahead. Deciding turn direction...")

            if left > right:
                cmd.angular.z = self.angular_velocity  # Turn left
                self.last_turn_direction = "left"
            elif right > left:
                cmd.angular.z = -self.angular_velocity  # Turn right
                self.last_turn_direction = "right"
            else:
                # If both sides are equal, alternate turns to avoid looping
                if self.last_turn_direction == "right":
                    cmd.angular.z = self.angular_velocity  # Turn left
                    self.last_turn_direction = "left"
                else:
                    cmd.angular.z = -self.angular_velocity  # Turn right
                    self.last_turn_direction = "right"

        # No obstacles, move forward
        else:
            self.get_logger().info("No obstacles, moving forward.")
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = 0.0  # No turning

        # Publish movement command
        self._pose_publisher.publish(cmd)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = TurtlebotMappingNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
