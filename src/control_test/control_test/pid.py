import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import numpy as np

# ==========================================
# TUNABLE PARAMETERS - PID
# ==========================================
KP = 1.8
KI = 0.025
KD = 0.8
DT = 0.1
MAX_THROTTLE = 3.0

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_node')
        self.current_v = 0.0
        self.target_v = 0.0
        self.last_error = 0.0
        self.integral_error = 0.0
        self.velocity_errors = []

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/state', self.state_cb, qos)
        self.create_subscription(Float32, '/target_velocity', self.target_cb, qos)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', qos)
        self.create_timer(DT, self.control_loop)

    def state_cb(self, msg): self.current_v = msg.twist.twist.linear.x
    def target_cb(self, msg): self.target_v = msg.data

    def control_loop(self):
        error = self.target_v - self.current_v
        self.velocity_errors.append(error)

        p_term = KP * error
        self.integral_error = np.clip(self.integral_error + (error * DT), -5.0, 5.0)
        i_term = KI * self.integral_error
        d_term = KD * (error - self.last_error) / DT
        self.last_error = error

        throttle = np.clip(p_term + i_term + d_term, -1.0, MAX_THROTTLE)
        self.throttle_pub.publish(Float32(data=float(throttle)))

        rmse = np.sqrt(np.mean(np.square(self.velocity_errors[-50:]))) if self.velocity_errors else 0.0
        self.get_logger().info(f"V: {self.current_v:.2f} | RMSE: {rmse:.3f}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PIDController())
    rclpy.shutdown()