import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry, Path 

#####################################################################
####           CONTROLLER - TUNED PARAMETERS ONLY               ####
#####################################################################

# --- Project Parameters ---
DT = 0.1
BICYCLE_LENGTH = 2.5
PUBLISH_RATE_HZ = 10

# --- Controller Constraints ---
MAX_THROTTLE = 1
MAX_STEERING_DEG = 35.0


# --- OPTUNA TUNED PARAMETERS - REPLACE WITH YOUR RESULTS ---
TUNED_PARAMETERS = {
    'kp_throttle': 1.8,
    'ki_throttle': 0.025,
    'kd_throttle': 0.8,
    'target_velocity': 3.5,
    'initial_lookahead': 2.5,
    'k_ld': 1.2,
    'ld_min': 2.0,
    'ld_max': 5.0,
}

# --- Stopping Mechanism Parameters ---
SLOWDOWN_START = 20.0
STOPPING_DISTANCE = 15.0
FINAL_STOP_DISTANCE = 1

#####################################################################
####           CONTROLLER IMPLEMENTATION                        ####
#####################################################################

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # Load ONLY tuned parameters
        self.load_tuned_parameters()
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # --- State and Path Variables ---
        self.current_state = {
            'x': 0.0, 
            'y': 0.0, 
            'yaw': 0.0, 
            'velocity': 0.0
        }
        self.waypoints = []
        self.is_path_received = False
        
        # --- PID Controller Variables ---
        self.last_error = 0.0
        self.integral_error = 0.0
        self.target_velocity = self.target_velocity_cruise
        
        # --- Stopping Mechanism ---
        self.has_stopped = False
        self.stopping_active = False
        self.stop_timer = 0.0
        
        # --- Metrics Tracking ---
        self.cross_track_error = 0.0
        self.max_cte = 0.0
        self.average_cte = 0.0
        self.cte_samples = 0
        self.total_distance = 0.0
        self.last_position = None
        
        # --- Path following ---
        self.last_target_index = 0
        
        # --- Subscriptions ---
        self.create_subscription(Odometry, '/state', self.stateCallback, qos_profile)
        self.create_subscription(Path, '/path', self.pathCallback, qos_profile)

        # --- Publishers ---
        self.steer_publisher = self.create_publisher(Float32, '/steer', qos_profile)
        self.throttle_publisher = self.create_publisher(Float32, '/throttle', qos_profile)
        
        # Control loop timer
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.control_loop)
        
        self.get_logger().info("🚗 CONTROLLER STARTED - Using Tuned Parameters")
        self.log_parameters()
        
    def load_tuned_parameters(self):
        """Load ONLY from TUNED_PARAMETERS - no manual fallback"""
        params = TUNED_PARAMETERS
        
        # PID Parameters
        self.Kp_throttle = params['kp_throttle']
        self.Ki_throttle = params['ki_throttle']
        self.Kd_throttle = params['kd_throttle']
        self.target_velocity_cruise = params['target_velocity']
        
        # Pure Pursuit Parameters
        self.Ld = params['initial_lookahead']
        self.K_ld = params['k_ld']
        self.Ld_min = params['ld_min']
        self.Ld_max = params['ld_max']
        
        # Stopping Parameters (fixed)
        self.SLOWDOWN_START = SLOWDOWN_START
        self.STOPPING_DISTANCE = STOPPING_DISTANCE
        self.FINAL_STOP_DISTANCE = FINAL_STOP_DISTANCE
        
    def log_parameters(self):
        """Log the parameters being used"""
        self.get_logger().info("🎯 ACTIVE PARAMETERS:")
        self.get_logger().info(f"   PID: KP={self.Kp_throttle}, KI={self.Ki_throttle}, KD={self.Kd_throttle}")
        self.get_logger().info(f"   Velocity: {self.target_velocity_cruise} m/s")
        self.get_logger().info(f"   Lookahead: {self.Ld_min}-{self.Ld_max} m (K={self.K_ld})")
        self.get_logger().info(f"   Stopping: Slow={self.SLOWDOWN_START}m, Stop={self.STOPPING_DISTANCE}m")
        
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stateCallback(self, state: Odometry):
        """Update current vehicle state"""
        self.current_state['x'] = state.pose.pose.position.x
        self.current_state['y'] = state.pose.pose.position.y
        self.current_state['yaw'] = self.quaternion_to_yaw(state.pose.pose.orientation)
        self.current_state['velocity'] = state.twist.twist.linear.x
        
        # Update total distance traveled
        current_pos = np.array([self.current_state['x'], self.current_state['y']])
        if self.last_position is not None:
            self.total_distance += np.linalg.norm(current_pos - self.last_position)
        self.last_position = current_pos

    def pathCallback(self, path: Path):
        """Receive and process path waypoints"""
        if len(path.poses) > 0:
            self.waypoints = path.poses
            self.is_path_received = True
            self.get_logger().info(f"📈 Path received: {len(self.waypoints)} points")

    def calculate_distance_to_goal(self):
        """Calculate distance to the final waypoint"""
        if not self.is_path_received or len(self.waypoints) == 0:
            return float('inf')
            
        last_waypoint = self.waypoints[-1].pose.position
        return math.sqrt(
            (last_waypoint.x - self.current_state['x'])**2 + 
            (last_waypoint.y - self.current_state['y'])**2
        )

    def adaptive_velocity_planning(self):
        """Velocity planning with stopping"""
        if not self.is_path_received or len(self.waypoints) == 0:
            self.target_velocity = self.target_velocity_cruise
            return
            
        distance_to_goal = self.calculate_distance_to_goal()
        
        # Three-stage stopping
        if distance_to_goal <= self.FINAL_STOP_DISTANCE:
            self.target_velocity = 0.0
            self.stopping_active = True
            if self.current_state['velocity'] < 0.1:
                self.stop_timer += DT
                if self.stop_timer >= 3.0 and not self.has_stopped:
                    self.get_logger().info("🛑 SUCCESS: Stopped at goal for 3 seconds!")
                    self.has_stopped = True
                
        elif distance_to_goal <= self.STOPPING_DISTANCE:
            self.stopping_active = True
            progress = (distance_to_goal - self.FINAL_STOP_DISTANCE) / (self.STOPPING_DISTANCE - self.FINAL_STOP_DISTANCE)
            self.target_velocity = 2.0 * progress
            
        elif distance_to_goal <= self.SLOWDOWN_START:
            self.stopping_active = False
            progress = distance_to_goal / self.SLOWDOWN_START
            self.target_velocity = self.target_velocity_cruise * (0.4 + 0.6 * progress)
            
        else:
            self.stopping_active = False
            self.target_velocity = self.target_velocity_cruise

    def pidController(self):
        """PID controller for throttle"""
        current_velocity = self.current_state['velocity']
        error = self.target_velocity - current_velocity

        # PID terms
        p_term = self.Kp_throttle * error
        self.integral_error += error * DT
        self.integral_error = max(min(self.integral_error, 0.2/self.Ki_throttle), -0.2/self.Ki_throttle)
        i_term = self.Ki_throttle * self.integral_error
        d_term = self.Kd_throttle * (error - self.last_error) / DT
        self.last_error = error

        # Combine
        throttle_output = p_term + i_term + d_term
        
        # Deadzone
        if abs(error) < 0.15:
            throttle_output *= 0.4
            
        throttle_output = np.clip(throttle_output, -MAX_THROTTLE, MAX_THROTTLE)
        
        return throttle_output

    def adaptive_lookahead(self):
        """Adaptive lookahead"""
        velocity = self.current_state['velocity']
        adaptive_ld = self.K_ld * velocity
        self.Ld = np.clip(adaptive_ld, self.Ld_min, self.Ld_max)
        
        # Reduce near goal
        distance_to_goal = self.calculate_distance_to_goal()
        if distance_to_goal < 12.0:
            goal_factor = max(0.4, distance_to_goal / 12.0)
            self.Ld *= goal_factor
            
        return self.Ld

    def searchTargetPoint(self):
        """Find target point on path"""
        if not self.is_path_received or len(self.waypoints) == 0:
            return -1
        
        current_x = self.current_state['x']
        current_y = self.current_state['y']
        
        start_idx = max(0, self.last_target_index)
        best_score = float('inf')
        target_index = -1
        
        for i in range(start_idx, min(start_idx + 25, len(self.waypoints))):
            pose_stamped = self.waypoints[i]
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            distance_score = abs(distance - self.Ld)
            progression_score = (i - self.last_target_index) * 0.08
            
            total_score = distance_score + progression_score
            
            if total_score < best_score and distance >= self.Ld * 0.4:
                best_score = total_score
                target_index = i
        
        if target_index == -1 and len(self.waypoints) > 0:
            target_index = min(self.last_target_index + 3, len(self.waypoints) - 1)
        
        if target_index != -1:
            self.last_target_index = target_index
            
        return target_index

    def purePursuit(self):
        """Pure Pursuit steering"""
        if not self.is_path_received or len(self.waypoints) == 0:
            return 0.0
        
        self.adaptive_lookahead()
        target_index = self.searchTargetPoint()
        
        if target_index == -1:
            return 0.0

        target_point = self.waypoints[target_index].pose.position
        dx = target_point.x - self.current_state['x']
        dy = target_point.y - self.current_state['y']
        
        # Transform to vehicle coordinates
        cos_yaw = math.cos(self.current_state['yaw'])
        sin_yaw = math.sin(self.current_state['yaw'])
        vehicle_x = dx * cos_yaw + dy * sin_yaw
        vehicle_y = -dx * sin_yaw + dy * cos_yaw
        
        Ld_actual = math.sqrt(vehicle_x**2 + vehicle_y**2)
        
        if Ld_actual < 0.1:
            return 0.0
        
        # Pure pursuit geometry
        curvature = 2.0 * vehicle_y / (Ld_actual**2)
        steering_rad = math.atan(curvature * BICYCLE_LENGTH)
        
        # Convert to degrees
        steering_deg = math.degrees(steering_rad)
        steering_deg = np.clip(steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG)
        
        return steering_deg

    def calculate_cross_track_error(self):
        """Calculate cross-track error"""
        if not self.is_path_received or len(self.waypoints) < 2:
            return 0.0
            
        x, y = self.current_state['x'], self.current_state['y']
        min_distance = float('inf')
        
        for i in range(max(0, self.last_target_index - 3), 
                      min(len(self.waypoints), self.last_target_index + 7) - 1):
            p1 = self.waypoints[i].pose.position
            p2 = self.waypoints[i+1].pose.position
            
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            segment_len_sq = dx*dx + dy*dy
            
            if segment_len_sq < 1e-6:
                continue
                
            t = ((x - p1.x) * dx + (y - p1.y) * dy) / segment_len_sq
            t = max(0, min(1, t))
            
            proj_x = p1.x + t * dx
            proj_y = p1.y + t * dy
            distance = math.sqrt((x - proj_x)**2 + (y - proj_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                
        return min_distance if min_distance != float('inf') else 0.0

    def control_loop(self):
        """Main control loop"""
        if not self.is_path_received:
            self.get_logger().info('⏳ Waiting for path...', throttle_duration_sec=2.0)
            return
            
        # Update planning and metrics
        self.adaptive_velocity_planning()
        self.cross_track_error = self.calculate_cross_track_error()
        self.max_cte = max(self.max_cte, self.cross_track_error)
        self.cte_samples += 1
        self.average_cte += (self.cross_track_error - self.average_cte) / self.cte_samples
        
        # Calculate controls
        throttle_command = self.pidController()
        steer_command = self.purePursuit()
        
        # Stopping logic
        if self.stopping_active and self.current_state['velocity'] < 0.2:
            throttle_command = min(throttle_command, -0.15)
        
        # Publish controls
        self.throttle_publisher.publish(Float32(data=throttle_command))
        self.steer_publisher.publish(Float32(data=steer_command))
        
        # Logging
        goal_dist = self.calculate_distance_to_goal()
        status = "🛑" if self.stopping_active else "🚗"
        if self.has_stopped:
            status = "🎯"
            
        self.get_logger().info(
            f'{status} S:{steer_command:5.1f}° T:{throttle_command:5.2f} '
            f'V:{self.current_state["velocity"]:4.1f}m/s '
            f'CTE:{self.cross_track_error:4.2f}m Goal:{goal_dist:5.1f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Controller shutdown requested")
    finally:
        # Final report
        controller.get_logger().info(
            f"🎯 FINAL: Max CTE: {controller.max_cte:.3f}m, "
            f"Avg CTE: {controller.average_cte:.3f}m, "
            f"Success: {'YES' if controller.has_stopped else 'NO'}"
        )
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()