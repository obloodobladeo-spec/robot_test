"""cmd_vel → 차동 구동 바퀴 목표 → ST-LINK VCP."""
import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import serial
from std_msgs.msg import Float64MultiArray, String

from .control import clamp, wheel_targets
from .protocol import command, parse_status


class SerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("robot_serial_bridge")
        defaults = {"port": "/dev/ttyACM0", "baudrate": 115200,
                    "wheel_separation_m": 0.0, "motor1_side": "unset",
                    "enable_motors": False, "max_linear_m_s": 0.2,
                    "max_angular_rad_s": 0.5, "max_wheel_m_s": 0.2,
                    "cmd_timeout_s": 0.3, "feedback_timeout_s": 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        def p(name: str):
            return self.get_parameter(name).value
        self.enabled = bool(p("enable_motors"))
        self.separation = float(p("wheel_separation_m"))
        self.side = str(p("motor1_side"))
        self.max_linear = float(p("max_linear_m_s"))
        self.max_angular = float(p("max_angular_rad_s"))
        self.max_wheel = float(p("max_wheel_m_s"))
        self.cmd_timeout = float(p("cmd_timeout_s"))
        self.feedback_timeout = float(p("feedback_timeout_s"))
        numbers = (self.separation, self.max_linear, self.max_angular, self.max_wheel,
                   self.cmd_timeout, self.feedback_timeout)
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("설정에 NaN/Inf를 사용할 수 없습니다.")
        if not (0 < self.max_linear <= 0.2 and 0 < self.max_wheel <= 0.2
                and 0 < self.max_angular <= 0.5
                and 0.05 <= self.cmd_timeout <= 0.5
                and 0.1 <= self.feedback_timeout <= 1.0):
            raise ValueError("속도 또는 타임아웃 설정이 허용 범위를 벗어났습니다.")
        if self.enabled and (self.separation <= 0 or self.side not in ("left", "right")):
            raise ValueError("구동 전 wheel_separation_m과 motor1_side를 설정하세요.")
        self.linear = self.angular = 0.0
        self.last_cmd = self.last_feedback = float("-inf")
        self.sequence = 0
        self.buffer = bytearray()
        self.discard_line = False
        self.link_failed = False
        self.status_pub = self.create_publisher(String, "robot/status", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, "robot/motor_speeds", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, qos)
        self.port = serial.Serial(str(p("port")), int(p("baudrate")), timeout=0,
                                  write_timeout=0.02, exclusive=True)
        try:
            self.port.reset_input_buffer()
            self.send(0, 0)
            self.timer = self.create_timer(0.05, self.tick)
        except (serial.SerialException, OSError):
            self.port.close()
            raise
        self.get_logger().info(
            f"USB 115200bps, 구동 {'활성' if self.enabled else '잠금'}, 바퀴 상한 200mm/s")

    def on_cmd(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.linear = self.angular = 0.0
            self.last_cmd = float("-inf")
            self.get_logger().error("유효하지 않은 cmd_vel: 정지")
            return
        self.linear = clamp(msg.linear.x, self.max_linear)
        self.angular = clamp(msg.angular.z, self.max_angular)
        self.last_cmd = time.monotonic()

    def send(self, motor1: int, motor2: int) -> None:
        self.sequence = (self.sequence + 1) & 65535
        data = command(self.sequence, motor1, motor2)
        if self.port.write(data) != len(data):
            raise serial.SerialException("부분 전송: 연결을 다시 시작하세요.")

    def consume_feedback(self, data: bytes) -> None:
        for value in data:
            if value == 10:
                if not self.discard_line:
                    try:
                        status = parse_status(bytes(self.buffer))
                    except ValueError:
                        pass
                    else:
                        self.last_feedback = time.monotonic()
                        self.wheel_pub.publish(Float64MultiArray(
                            data=[status.motor1_mm_s / 1000.0, status.motor2_mm_s / 1000.0]))
                        self.status_pub.publish(String(data=json.dumps({
                            "sequence": status.sequence, "motor1_mm_s": status.motor1_mm_s,
                            "motor2_mm_s": status.motor2_mm_s,
                            "motor1_duty_percent": status.motor1_duty_centi / 100.0,
                            "motor2_duty_percent": status.motor2_duty_centi / 100.0,
                            "stm32_timeout": status.timed_out, "enabled": self.enabled})))
                self.buffer.clear()
                self.discard_line = False
            elif not self.discard_line:
                if len(self.buffer) >= 95:
                    self.buffer.clear()
                    self.discard_line = True
                else:
                    self.buffer.append(value)

    def tick(self) -> None:
        if self.link_failed:
            return
        try:
            self.consume_feedback(self.port.read(min(self.port.in_waiting, 512)))
            now = time.monotonic()
            if now - self.last_feedback >= self.feedback_timeout:
                # 피드백 단절 뒤 이전 이동 명령으로 자동 재출발하지 않습니다.
                self.last_cmd = float("-inf")
            valid = (self.enabled and now - self.last_cmd < self.cmd_timeout
                     and now - self.last_feedback < self.feedback_timeout)
            if valid:
                left, right = wheel_targets(self.linear, self.angular,
                                            self.separation, self.max_wheel)
                m1, m2 = (left, right) if self.side == "left" else (right, left)
            else:
                m1 = m2 = 0
            self.send(m1, m2)
        except (serial.SerialException, OSError) as exc:
            self.link_failed = True
            self.linear = self.angular = 0.0
            self.last_cmd = float("-inf")
            self.get_logger().error(f"USB 통신 중단: {exc}. STM32 타임아웃 정지 후 노드를 재시작하세요.")
            self.port.close()

    def close(self) -> None:
        if self.port.is_open:
            try:
                self.send(0, 0)
            except (serial.SerialException, OSError):
                pass
            finally:
                self.port.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
