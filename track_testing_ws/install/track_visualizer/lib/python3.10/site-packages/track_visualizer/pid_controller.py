import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # --- PID GAINS ---
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.target_speed = 0.5
        # -----------------
        
        self.path = []
        self.current_pose = None
        self.prev_error = 0.0
        self.integral = 0.0

        # Subscriber for the path (Must use Transient Local QoS to match publisher)
        qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/track_path', self.path_cb, qos_map)
        
        # Subscriber for robot position
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control Loop Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("PID Controller waiting for path and odom...")

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f"PID received path: {len(self.path)} points")

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw(self, q):
        """Converts quaternion to yaw angle."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_cte(self, x, y, yaw):
        if not self.path: return 0.0
        
        # 1. Find the closest point on the path
        closest_dist = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(px - x, py - y)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
                
        # 2. Calculate heading error relative to that point
        path_point = self.path[closest_idx]
        desired_heading = math.atan2(path_point[1] - y, path_point[0] - x)
        
        error = desired_heading - yaw
        
        # 3. Normalize error to [-pi, pi]
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        
        return error

    def control_loop(self):
        if not self.path or not self.current_pose:
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw(self.current_pose.orientation)

        # PID Calculation
        error = self.get_cte(x, y, yaw)
        
        P = self.kp * error
        self.integral += error
        I = self.ki * self.integral
        D = self.kd * (error - self.prev_error)
        
        steering = P + I + D
        self.prev_error = error

        # Publish Command
        cmd = Twist()
        cmd.linear.x = self.target_speed
        cmd.angular.z = max(min(steering, 1.0), -1.0) # Clamp between -1.0 and 1.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()