import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

import time
import math
import serial

class CmdVelReceiver(Node):
    def __init__(self):
        super().__init__("cmd_vel_receiver")

        self.linear_mm_s = 0
        self.angular_mrad_s = 0

        self.port = serial.Serial(
                port="/dev/ttyACM0",
                baudrate=115200,
                timeout=0,
                write_timeout=0.1,
            )

        self.port.reset_input_buffer()

        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.on_cmd_vel,
            10,
        )

        self.last_cmd_time = None
        self.cmd_time_out = 0.5

        self.rx_buffer = bytearray()

        self.left_speed_mm_s = 0
        self.right_speed_mm_s = 0
        self.x_mm = 0
        self.y_mm = 0
        self.yaw_rad = 0

        self.wheel_speed_pub = self.create_publisher(Float64MultiArray, '/wheel_speeds', 10)

        self.wheel_track_m = 0.230

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.1, self.send_command)

    def on_cmd_vel(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if not(math.isfinite(linear) and math.isfinite(angular)):
            self.linear_mm_s = 0
            self.angular_mrad_s = 0
            self.last_cmd_time = None

            self.get_logger().warning("Invalid velocity command: stopping")
            return

        linear = max(-0.2, min(linear, 0.2))
        angular = max(-0.5, min(angular, 0.5))

        self.linear_mm_s = round(linear * 1000.0)
        self.angular_mrad_s = round(angular * 1000.0)

        self.last_cmd_time = time.monotonic()

    def send_command(self):
        now = time.monotonic()

        if (self.last_cmd_time is None or now - self.last_cmd_time >= self.cmd_time_out):
            self.linear_mm_s = 0
            self.angular_mrad_s = 0

        command = (
            f"V={self.linear_mm_s} "
            f"W={self.angular_mrad_s}\n"
        )

        self.port.write(command.encode("ascii"))
        self.get_logger().info(command.strip())

        self.receive_feedback()

    def receive_feedback(self):
        received = self.port.read(256)
        self.rx_buffer.extend(received)

        while b"\n" in self.rx_buffer:
            line, _, remaining = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remaining)

            text = line.decode(
                "ascii",
                errors="replace",
            ).strip()

            try:
                values = self.parse_feedback(text)

                left = values["L"]
                right = values["R"]
                x = values["X"]
                y = values["Y"]
                yaw = values["A"] / 1000.0

            except (ValueError, KeyError):
                self.get_logger().warning(
                    f"Invalid feedback: {text}"
                )
                continue

            self.left_speed_mm_s = left
            self.right_speed_mm_s = right
            self.x_mm = x
            self.y_mm = y
            self.yaw_rad = yaw

            self.publish_odom()

            msg = Float64MultiArray()

            msg.data = [
                self.left_speed_mm_s / 1000.0,
                self.right_speed_mm_s / 1000.0,
            ]

            self.wheel_speed_pub.publish(msg)

            self.get_logger().info(
                f"L={self.left_speed_mm_s} mm/s, "
                f"R={self.right_speed_mm_s} mm/s, "
                f"X={self.x_mm} mm, Y={self.y_mm} mm "
                f"A={self.yaw_rad:.3f} rad"
            )

    def parse_feedback(self, text):
        values = {}

        for item in text.split():
            key, value = item.split("=")
            values[key] = int(value)

        return values

    def publish_odom(self):
        msg = Odometry()

        # mcu의 시간을 받지 못함으로 node 시간을 활용
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # 위치 stm32의 mm값을 m로 변환
        msg.pose.pose.position.x = self.x_mm / 1000.0
        msg.pose.pose.position.y = self.y_mm / 1000.0

        # 평면에서 방향각을 쿼터니안으로 계산 
        msg.pose.pose.orientation.z = math.sin(self.yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw_rad / 2.0)

        left_m_s = self.left_speed_mm_s / 1000.0
        right_m_s = self.right_speed_mm_s / 1000.0

        # 실제 차 속도 계산 (두바퀴 속도의 평균), z : 두 바퀴 속도 / 휠 크기
        msg.twist.twist.linear.x = (left_m_s + right_m_s) / 2.0
        msg.twist.twist.angular.z = ((right_m_s - left_m_s) / self.wheel_track_m)

        self.odom_pub.publish(msg)

        transform = TransformStamped()

        # odom과 동일한 시각, 위치, 좌표계를 사용
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.port.write(b"V=0 W=0\n")
        finally:
            node.port.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
