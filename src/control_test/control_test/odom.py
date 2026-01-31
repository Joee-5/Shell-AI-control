import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from visualization_msgs.msg import Marker
import tf2_ros
import math
import numpy as np

# ==============================================================================
# HARDWARE SPECIFICATIONS
# ==============================================================================
WHEELBASE = 1.8           #
MAX_SPEED = 4.17          #
MAX_ACCEL = 3.0           #
STEER_LIMIT_DEG = 20.0    #
VEHICLE_L, VEHICLE_W = 2.4, 1.0 #
DT = 0.1

class AckermannSimNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.steer_angle, self.throttle = 0.0, 0.0
        self.initialized = False

        self.odom_pub = self.create_publisher(Odometry, '/state', 10)
        self.marker_pub = self.create_publisher(Marker, '/vehicle_marker', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.create_subscription(Float32, '/throttle', self.throttle_cb, 10)
        self.create_subscription(Float32, '/steer', self.steer_cb, 10)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.create_timer(DT, self.update_physics)
        self.get_logger().info("🏎️ Ackermann Physics Simulator Active")

    def path_cb(self, msg):
        if not self.initialized and len(msg.poses) > 0:
            start = msg.poses[0].pose.position
            self.x, self.y = start.x, start.y
            if len(msg.poses) > 1:
                p2 = msg.poses[1].pose.position
                self.yaw = math.atan2(p2.y - self.y, p2.x - self.x)
            self.initialized = True
            self.get_logger().info("📍 Position Snapped to Path Start")

    def throttle_cb(self, msg): self.throttle = np.clip(msg.data, -1.0, 1.0)
    def steer_cb(self, msg): self.steer_angle = math.radians(np.clip(msg.data, -STEER_LIMIT_DEG, STEER_LIMIT_DEG))

    def update_physics(self):
        if not self.initialized: return

        # 1. Velocity Update (with 3.0 m/s^2 acceleration limit)
        target_v = self.throttle * MAX_SPEED
        accel = np.clip((target_v - self.v) / DT, -MAX_ACCEL, MAX_ACCEL)
        self.v = np.clip(self.v + accel * DT, 0.0, MAX_SPEED)

        # 2. Ackermann Kinematics
        # Car only turns if it has forward velocity
        if abs(self.v) > 0.01:
            # Heading Change = (v / L) * tan(delta)
            self.yaw += (self.v / WHEELBASE) * math.tan(self.steer_angle) * DT
            # Position Change
            self.x += self.v * math.cos(self.yaw) * DT
            self.y += self.v * math.sin(self.yaw) * DT

        self.publish_all()

    def publish_all(self):
        t = self.get_clock().now().to_msg()
        q = Quaternion(x=0.0, y=0.0, z=math.sin(self.yaw/2), w=math.cos(self.yaw/2))
        
        # TF Broadcast (Solves Fixed Frame issue)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = t, 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

        # Odometry for Controllers
        odom = Odometry()
        odom.header = tf.header
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v
        self.odom_pub.publish(odom)

        # 3D Model Marker
        m = Marker()
        m.header.frame_id, m.header.stamp, m.type = "map", t, Marker.CUBE
        m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation = self.x, self.y, 0.25, q
        m.scale = Vector3(x=VEHICLE_L, y=VEHICLE_W, z=0.5)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 1.0
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AckermannSimNode())
    rclpy.shutdown()
