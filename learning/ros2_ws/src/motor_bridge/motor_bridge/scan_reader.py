import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanReader(Node):

    def __init__(self):
        super().__init__('scan_reader')

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        target_angle = math.pi / 2
        index = round((target_angle - msg.angle_min) / msg.angle_increment)

        if not (0 <= index < len(msg.ranges)):
            self.get_logger().warning(
                f'Index out of range: {index}',
                throttle_duration_sec=1.0
            )
            return

        distance = msg.ranges[index]

        if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
            self.get_logger().info(
                f'index={index}, distance={distance:.3f} m',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f'Invalid distance: {distance:.3f} m',
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


if __name__ == '__main__':
    main()
