import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path # Added Path import
import numpy as np
import threading
import math # Added for distance calculation
import matplotlib.pyplot as plt

# ==========================================
# STABILIZED PARAMETERS
# trial 6.747 0.877 0.21 
# ==========================================
KP = 6.747           
KI = 0.877            
KD = 0.21             
control_period = 0.1 
MAX_THROTTLE = 1.0   
MAX_LOG_LEN = 200    
STOP_TOLERANCE = 0.2  # [m] Distance to final point to kill throttle

class PIDController(Node):
    def __init__(self): 
        super().__init__('pid_node') 

        self.current_v = 0.0
        self.target_v = 0.0
        self.x, self.y = 0.0, 0.0 # Tracking position
        self.last_error = 0.0
        self.integral_error = 0.0
        self.velocity_errors = []
        self.final_point = None # Stores the end of the track

        self.time_log = []
        self.target_log = []
        self.actual_log = []
        self.error_log = []
        self.lock = threading.Lock()
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Odometry, '/state', self.state_cb, qos)
        self.create_subscription(Float32, '/target_velocity', self.target_cb, qos)
        self.create_subscription(Path, '/path', self.path_cb, qos) # Added path sub
        self.throttle_pub = self.create_publisher(Float32, '/throttle', qos)
        self.create_timer(control_period, self.control_loop)

        threading.Thread(target=self.live_plot_thread, daemon=True).start()

    def state_cb(self, msg):
        self.x = msg.pose.pose.position.x # Update position
        self.y = msg.pose.pose.position.y
        alpha = 0.2  
        raw_v = msg.twist.twist.linear.x
        self.current_v = alpha * raw_v + (1 - alpha) * self.current_v

    def target_cb(self, msg): 
        self.target_v = msg.data

    def path_cb(self, msg):
        # Extract the final waypoint from the path
        if len(msg.poses) > 0:
            self.final_point = msg.poses[-1].pose.position

    def control_loop(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0:
            return

        # --- STOPPING LOGIC ---
        if self.final_point is not None:
            dist_to_end = math.sqrt((self.final_point.x - self.x)**2 + (self.final_point.y - self.y)**2)
            if dist_to_end < STOP_TOLERANCE:
                self.get_logger().info("🏁 End reached: Cutting throttle")
                self.throttle_pub.publish(Float32(data=0.0))
                return # Skip PID calculation
        # ----------------------

        error = self.target_v - self.current_v
        self.velocity_errors.append(error)
        
        self.integral_error = np.clip(self.integral_error + (error * dt), -2.0, 2.0)
        
        p_term = KP * error
        i_term = KI * self.integral_error
        d_term = KD * (error - self.last_error) / dt
        self.last_error = error

        with self.lock:
            self.time_log.append(current_time)
            self.target_log.append(self.target_v)
            self.actual_log.append(self.current_v)
            self.error_log.append(error)
            
            self.time_log = self.time_log[-MAX_LOG_LEN:]
            self.target_log = self.target_log[-MAX_LOG_LEN:]
            self.actual_log = self.actual_log[-MAX_LOG_LEN:]
            self.error_log = self.error_log[-MAX_LOG_LEN:]

        throttle = np.clip(p_term + i_term + d_term, -1.0, MAX_THROTTLE)
        self.throttle_pub.publish(Float32(data=float(throttle)))

        rmse = np.sqrt(np.mean(np.square(self.velocity_errors[-50:]))) if self.velocity_errors else 0.0
        self.get_logger().info(f"V: {self.current_v:.2f} | RMSE: {rmse:.3f}")

    def live_plot_thread(self):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        while rclpy.ok():
            with self.lock:
                if len(self.time_log) > 1:
                    ax1.clear()
                    ax2.clear()
                    ax1.plot(self.time_log, self.target_log, label="Target (4.0 m/s)")
                    ax1.plot(self.time_log, self.actual_log, label="Actual Speed")
                    ax1.set_ylabel("Speed (m/s)")
                    ax1.legend()
                    ax1.grid()
                    ax2.plot(self.time_log, self.error_log, color='r')
                    ax2.set_ylabel("Error (m/s)")
                    ax2.grid()
            plt.pause(0.1)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PIDController())
    rclpy.shutdown()