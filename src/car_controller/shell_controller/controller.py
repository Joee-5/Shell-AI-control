import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import matplotlib.pyplot as plt
import threading


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Initialization
        self.time_log = []
        self.target_log = []
        self.actual_log = []
        self.error_log = []

        # PID constants
        self.kp = 40.0
        self.ki = 0.101
        self.kd = 0.0

        # State
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Dynamic target speed (updated from /target_speed)
        self.target_speed = 1.5  # default in case topic is not received yet
        self.start_time = time.time()

        # ROS subscriptions and publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/target_speed2', self.target_speed_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Start live plot thread
        threading.Thread(target=self.live_plot_thread, daemon=True).start()

    def target_speed_callback(self, msg):
        self.target_speed = msg.data
        self.get_logger().info(f"Updated target speed: {self.target_speed:.2f} m/s")

    def odom_callback(self, msg):
        now = time.time()
        elapsed_time = now - self.start_time

        # Current vehicle speed
        current_speed = msg.twist.twist.linear.x
        target_speed = self.target_speed

        # PID error
        error = target_speed - current_speed
        error_percent = (error / target_speed) * 100 if target_speed != 0 else 0.0
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time

        # PID terms
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Command output (clamped to ≥ 0)
        cmd = Twist()
        cmd.linear.x = max(0.0, output)
        self.publisher.publish(cmd)

        # Logging
        self.get_logger().info(
            f"Target: {target_speed:.2f}, Actual: {current_speed:.2f}, Output: {output:.2f}, Error: {error_percent:.2f}%"
        )

        self.time_log.append(elapsed_time)
        self.target_log.append(target_speed)
        self.actual_log.append(current_speed)
        self.error_log.append(error)

        self.previous_error = error
        self.last_time = current_time

    def live_plot_thread(self):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        while True:
            if len(self.time_log) > 1:
                ax1.clear()
                ax2.clear()

                ax1.plot(self.time_log, self.target_log, label="Target Speed")
                ax1.plot(self.time_log, self.actual_log, label="Actual Speed")
                ax1.set_ylabel("Speed (m/s)")
                ax1.set_title("Target vs Actual Speed")
                ax1.legend()
                ax1.grid()

                ax2.plot(self.time_log, self.error_log, label="Speed Error", color='r')
                ax2.set_ylabel("Error (m/s)")
                ax2.set_xlabel("Time (s)")
                ax2.set_title("Speed Error Over Time")
                ax2.grid()

                plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()