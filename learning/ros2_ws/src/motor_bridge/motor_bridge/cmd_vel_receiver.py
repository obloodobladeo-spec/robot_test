import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import time
import math
import serial


# covariance (2026-11-21 실측 근거, 단위 SI). 6x6 행 우선: x y z roll pitch yaw.
# /odom: 위치·yaw는 엔코더 적분(직진 30 cm에서 IMU 대비 yaw 1~2° 차이, 회전 시 ~1%).
ODOM_POSE_VAR = (1e-3, 1e-3, 1e6, 1e6, 1e6, 2.5e-3)
# 속도: 50 mm/s 주행 중 바퀴 속도 흔들림 약 ±5 mm/s, 각속도 엔코더-IMU 차 평균 ~11 mrad/s.
# vy는 차동 구동이라 0(작은 분산), z·roll·pitch는 2D라 사용 안 함(큰 분산).
ODOM_TWIST_VAR = (1e-4, 1e-6, 1e6, 1e6, 1e6, 4e-4)
# /imu/data: Game Rotation Vector 정지 노이즈 ~0.005°, 드리프트 ~0.1~0.3°/min.
IMU_ORIENTATION_VAR = (1e-4, 1e-4, 1e-4)
# 보정 자이로: 정지 표준편차 ~1 mrad/s, 모터 구동 중 ~8 mrad/s.
IMU_ANGULAR_VELOCITY_VAR = (1e-4, 1e-4, 1e-4)


def diagonal(values):
    size = len(values)
    matrix = [0.0] * (size * size)
    for i, value in enumerate(values):
        matrix[i * size + i] = value
    return matrix


class CmdVelReceiver(Node):
    def __init__(self):
        super().__init__("cmd_vel_receiver")

        self.linear_mm_s = 0
        self.angular_mrad_s = 0

        serial_port = self.declare_parameter(
            "serial_port", "/dev/ttyACM0").value
        baudrate = self.declare_parameter("baudrate", 115200).value

        self.port = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
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

        # STM32 WHEEL_TRACK_MM(243.1f)과 같은 값을 유지한다.
        # 실측 바퀴 간격 234 mm가 아니라 제자리 회전으로 보정한 실효 윤거다.
        self.wheel_track_m = self.declare_parameter(
            "wheel_track_m", 0.2431).value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # robot_localization EKF가 odom->base_link TF를 낼 때는 False로 둔다.
        self.publish_tf = self.declare_parameter("publish_tf", True).value
        self.tf_broadcaster = TransformBroadcaster(self)

        # STM32 telemetry의 IS=1 IMU 필드(BNO085, 센서 자체 축)를 /imu/data로 발행
        self.imu_frame_id = self.declare_parameter(
            "imu_frame_id", "imu_link").value
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.last_seq = None

        # 명령 송신은 STM32 제어 주기와 같은 100 ms.
        self.create_timer(0.1, self.send_command)
        # 수신은 따로 20 ms마다 확인한다. 100 ms 송신 타이머에서 같이 읽으면
        # MCU telemetry 주기와 위상이 겹칠 때 최대 100 ms 늦게 발행된다.
        self.create_timer(0.02, self.receive_feedback)

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
        self.get_logger().debug(command.strip())

    def receive_feedback(self):
        # 20 ms 동안 들어오는 양은 telemetry 한 줄(최대 약 190바이트) 이하이므로 여유 있게 읽는다.
        received = self.port.read(1024)
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

            self.check_sequence(values.get("N"))
            self.publish_imu(values)

            self.get_logger().info(
                f"L={self.left_speed_mm_s} mm/s, "
                f"R={self.right_speed_mm_s} mm/s, "
                f"X={self.x_mm} mm, Y={self.y_mm} mm "
                f"A={self.yaw_rad:.3f} rad "
                f"IMU={values.get('IS', 0)}",
                throttle_duration_sec=1.0,
            )

    FEEDBACK_KEYS = (
        "L", "R", "X", "Y", "A",          # 필수: 바퀴 속도·오도메트리
        "T", "N",                         # MCU 시각(ms), 순번
        "IS", "QX", "QY", "QZ", "QW",     # IMU 유효 여부, 쿼터니언 x10000
        "GX", "GY", "GZ", "QA", "GA",     # 각속도 mrad/s, 정확도 0~3
    )
    IMU_KEYS = ("QX", "QY", "QZ", "QW", "GX", "GY", "GZ")

    def parse_feedback(self, text):
        # 알려진 key만 해석하고, 이후 추가될 필드는 무시한다.
        values = {}

        for item in text.split():
            key, sep, value = item.partition("=")

            if not sep:
                raise ValueError(f"no '=' in {item!r}")

            if key in self.FEEDBACK_KEYS:
                values[key] = int(value)

        return values

    def check_sequence(self, seq):
        if seq is None:
            return

        if self.last_seq is not None and seq != self.last_seq + 1:
            self.get_logger().warning(
                f"Telemetry sequence jump: {self.last_seq} -> {seq}",
                throttle_duration_sec=1.0,
            )

        self.last_seq = seq

    def publish_imu(self, values):
        if values.get("IS") != 1:
            return

        if not all(key in values for key in self.IMU_KEYS):
            self.get_logger().warning(
                "IMU flag set but fields missing",
                throttle_duration_sec=1.0,
            )
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id

        msg.orientation.x = values["QX"] / 10000.0
        msg.orientation.y = values["QY"] / 10000.0
        msg.orientation.z = values["QZ"] / 10000.0
        msg.orientation.w = values["QW"] / 10000.0

        msg.angular_velocity.x = values["GX"] / 1000.0
        msg.angular_velocity.y = values["GY"] / 1000.0
        msg.angular_velocity.z = values["GZ"] / 1000.0

        # 가속도는 보내지 않으므로 REP-145에 따라 covariance[0] = -1로 표시한다.
        msg.orientation_covariance = diagonal(IMU_ORIENTATION_VAR)
        msg.angular_velocity_covariance = diagonal(IMU_ANGULAR_VELOCITY_VAR)
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

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

        msg.pose.covariance = diagonal(ODOM_POSE_VAR)
        msg.twist.covariance = diagonal(ODOM_TWIST_VAR)

        self.odom_pub.publish(msg)

        if not self.publish_tf:
            return

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
