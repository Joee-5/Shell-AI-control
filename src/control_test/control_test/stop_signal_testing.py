import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class StopSignalManualPublisher(Node):
    def __init__(self):
        super().__init__('stop_signal_manual')
        self.pub = self.create_publisher(Float32, '/stop_signal', 10)
        self.get_logger().info("🛑 Manual Stop Signal Publisher Ready")
        self.run_loop()

    def run_loop(self):
        try:
            while rclpy.ok():
                cmd = input("Press ENTER to send STOP signal, or 'q' then ENTER to quit: ")
                if cmd.lower() == 'q':
                    break
                msg = Float32()
                msg.data = 1.0
                self.pub.publish(msg)
                self.get_logger().info("🛑 Stop signal sent!")
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = StopSignalManualPublisher()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
