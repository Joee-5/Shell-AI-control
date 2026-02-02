import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path

# ==============================================================================
# TUNABLE PARAMETERS
# ==============================================================================
WHEELBASE = 1.8           #
MAX_STEER_DEG = 20.0      #
CRUISE_VELOCITY = 4.0     #
K_LD = 0.7                
LD_MIN = 1.6              
LD_MAX = 5.8              # Added variable
STOP_DIST = 15.0         
GOAL_TOLERANCE = 0.1
DECEL_RATE = 0.5    
# ==============================================================================

class SprintPurePursuitNode(Node):
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

        self.stop_signal = False
        self.create_subscription(Float32,'/stop_signal', self.stop_cb, qos)
        self.get_logger().info("Stopping at Final Point.")

    def state_cb(self, msg):
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_cb(self, msg): 
        self.waypoints = msg.poses

    def stop_cb(self, msg):
        self.stop_signal = bool(msg.data)  # True if data != 0
        if self.stop_signal:
            self.get_logger().info("Stop signal received!")


    def control_loop(self):
        if not self.waypoints: return
        
        # 1. Adaptive Lookahead
        ld = np.clip(K_LD * self.v, LD_MIN, LD_MAX)

        # 2. Find target point
        for i in range(self.last_idx, len(self.waypoints)):
            wp = self.waypoints[i].pose.position
            if math.sqrt((wp.x - self.x)**2 + (wp.y - self.y)**2) >= ld:
                self.last_idx = i
                break
        
        tp = self.waypoints[self.last_idx].pose.position

        # 3. Sprint Stopping Logic
        final_wp = self.waypoints[-1].pose.position
        dist_to_end = math.sqrt((final_wp.x - self.x)**2 + (final_wp.y - self.y)**2)
        
        if self.stop_signal:
            target_vel = max(self.v - DECEL_RATE * 0.1, 0.0)

        if dist_to_end < GOAL_TOLERANCE:
            target_vel = 0.0
        elif dist_to_end < STOP_DIST:
            target_vel = max(math.sqrt(dist_to_end / STOP_DIST) * CRUISE_VELOCITY, 0.2)
        else:
            target_vel = float(CRUISE_VELOCITY)
        
        # 4. Steering Calculation
        dx, dy = tp.x - self.x, tp.y - self.y
        local_y = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        steer_rad = math.atan2(2.0 * WHEELBASE * local_y, ld**2)
        final_steer = np.clip(math.degrees(steer_rad), -MAX_STEER_DEG, MAX_STEER_DEG)
        
        self.steer_pub.publish(Float32(data=float(final_steer)))
        self.target_vel_pub.publish(Float32(data=float(target_vel)))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SprintPurePursuitNode())
    rclpy.shutdown()