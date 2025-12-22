import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from kinematic_bicycle.bicycle_model import Car  # Fixed import

def main(args=None):
    rclpy.init(args=args)
    car = Car(np.array([0,0,0,0]), 0.1, 2.5)  # initial state: [x, y, yaw, steering]
    rclpy.spin(car)
    car.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()