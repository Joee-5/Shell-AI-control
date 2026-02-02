import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import optuna
import time
import numpy as np

class PIDTunerNode(Node):
    def __init__(self):
        super().__init__('pid_tuner_node')
        self.current_v = 0.0
        self.velocity_history = []
        
        # Monitor the vehicle state
        self.create_subscription(Odometry, '/state', self.state_cb, 10)
        self.get_logger().info("🔍 Tuner Node Active: Waiting for Evaluation...")

    def state_cb(self, msg):
        self.current_v = msg.twist.twist.linear.x
        self.velocity_history.append(self.current_v)

def objective(trial):
    # Recommend new parameters for the 110kg mass
    kp = trial.suggest_float("KP", 1.0, 10.0)
    ki = trial.suggest_float("KI", 0.01, 2.0)
    kd = trial.suggest_float("KD", 0.1, 1.5)
    
    # Initialize ROS2 for this trial
    rclpy.init()
    tuner_node = PIDTunerNode()
    
    # Evaluation Period (Simulate 10 seconds of driving)
    start_time = time.time()
    while (time.time() - start_time) < 10.0:
        rclpy.spin_once(tuner_node, timeout_sec=0.1)
    
    # Metric: Maximize average velocity, minimize RMSE
    if len(tuner_node.velocity_history) < 10:
        score = -100.0
    else:
        avg_v = np.mean(tuner_node.velocity_history)
        # Target is 4.0 m/s
        error = np.abs(4.0 - avg_v)
        score = avg_v - (error * 5.0) 

    tuner_node.destroy_node()
    rclpy.shutdown()
    return score

def main():
    study = optuna.create_study(direction="maximize")
    study.optimize(objective, n_trials=20) # 20 trial runs
    
    print("\n🏁 OPTUNA RECOMMENDATIONS:")
    print(study.best_params)

if __name__ == '__main__':
    main()