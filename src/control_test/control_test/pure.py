import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path

# ==============================================================================
# TUNABLE PARAMETERS - HARDWARE CONSTRAINED
# ==============================================================================
WHEELBASE = 1.8           #
MAX_STEER_DEG = 20.0      #
CRUISE_VELOCITY = 3.5     # Lowered speed to guarantee path tracking success

# AGGRESSIVE TUNING: Shortening these stops the "Big Circle" behavior
K_LD = 0.7                # How much LD scales with speed
LD_MIN = 1.0              # Minimum lookahead (Car reacts faster to nearby points)
LD_MAX = 3.7

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_node')
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.waypoints, self.last_idx = [], 0
        
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10, durability=DurabilityPolicy.VOLATILE)
        self.steer_pub = self.create_publisher(Float32, '/steer', qos)
        self.target_vel_pub = self.create_publisher(Float32, '/target_velocity', qos)
        
        self.create_subscription(Odometry, '/state', self.state_cb, qos)
        self.create_subscription(Path, '/path', self.path_cb, qos)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🎯 Pure Pursuit: Aggressive Tuning Enabled")

    def state_cb(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_cb(self, msg): 
        self.waypoints = msg.poses

    def control_loop(self):
        if not self.waypoints: return
        
        # 1. Dynamic Lookahead calculation
        ld = np.clip(K_LD * self.v, LD_MIN, LD_MAX)

        # 2. Find the target point index
        # We look for the first point that is at least 'ld' distance away
        found_target = False
        for i in range(self.last_idx, len(self.waypoints)):
            wp = self.waypoints[i].pose.position
            dist = math.sqrt((wp.x - self.x)**2 + (wp.y - self.y)**2)
            if dist >= ld:
                self.last_idx = i
                found_target = True
                break
        
        # If we reached the end or can't find a point far enough, target the last point
        tp = self.waypoints[self.last_idx].pose.position
        
        # 3. Coordinate Transformation (World to Vehicle Frame)
        dx, dy = tp.x - self.x, tp.y - self.y
        
        # This determines if the target is to the left (+) or right (-)
        local_y = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        
        # 4. Pure Pursuit Steering Formula for Ackermann
        # Angle = atan(2 * L * local_y / ld^2)
        steer_rad = math.atan2(2.0 * WHEELBASE * local_y, ld**2)
        steer_deg = math.degrees(steer_rad)
        
        # Apply hardware limit: +/- 20 degrees
        final_steer = np.clip(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
        
        # 5. Publish
        self.steer_pub.publish(Float32(data=float(final_steer)))
        self.target_vel_pub.publish(Float32(data=float(CRUISE_VELOCITY)))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PurePursuitNode())
    rclpy.shutdown()