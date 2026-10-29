import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class ScanReader(Node):
    def __init__(self):
        super().__init__("scan_reader")

        self.subscription = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.get_logger().info(
            f"frame={msg.header.frame_id}, points={len(msg.ranges)}",
            throttle_duration_sec=1.0,
        )

def main(args=None):
    rclpy.init(args=args)
    node = ScanReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()