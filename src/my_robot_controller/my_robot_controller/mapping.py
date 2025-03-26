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
        
        # Initialize obstacle detection parameters
        self.obstacle_threshold = 1.0  # Minimum distance to obstacles (meters)
        self.angular_velocity = 0.5   # Turn speed
        self.linear_velocity = 0.3    # Forward speed
        self.reverse_velocity = -0.2  # Backward speed
        self.rotation_time = 3        # Time to rotate in seconds to avoid continuous spinning

        # Timer for avoiding continuous spinning
        self.last_rotation_time = time.time()

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Extract directional distances from the laser scan data
        front = min(scan.ranges[:15] + scan.ranges[-15:])  # Front (0-30 degrees)
        left = min(scan.ranges[60:120])  # Left (60-120 degrees)
        right = min(scan.ranges[240:300])  # Right (240-300 degrees)

        # Log the distances for debugging
        self.get_logger().info(f"Front: {front}, Left: {left}, Right: {right}")

        current_time = time.time()

        # If the robot is stuck, try to move backwards and rotate
        if front < self.obstacle_threshold and left < self.obstacle_threshold and right < self.obstacle_threshold:
            self.get_logger().info("Robot is stuck, backing up and rotating...")
            cmd.linear.x = self.reverse_velocity  # Move backwards
            cmd.angular.z = self.angular_velocity  # Rotate to find a clear path
            self.last_rotation_time = current_time  # Start a new rotation timer

        # If the robot has been rotating too long, stop the rotation and move forward
        elif current_time - self.last_rotation_time > self.rotation_time:
            self.get_logger().info("Rotation time exceeded, moving forward...")
            cmd.linear.x = self.linear_velocity  # Move forward
            cmd.angular.z = 0.0  # Stop turning

        # If there is an obstacle ahead, try turning
        elif front < self.obstacle_threshold:  # Obstacle ahead
            self.get_logger().info("Obstacle detected ahead. Avoiding...")
            # Check which side has more space to turn
            if left > right:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_velocity  # Turn left
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.angular_velocity  # Turn right

        # If no obstacles detected, move forward
        else:
            self.get_logger().info("No obstacle ahead. Moving forward...")
            cmd.linear.x = self.linear_velocity  # Move forward
            cmd.angular.z = 0.0  # No turning

        # Publish the movement command
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



