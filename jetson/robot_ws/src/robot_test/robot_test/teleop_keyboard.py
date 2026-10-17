"""SSH TTY에서 실행하는 TurtleBot3 방식 증분 키보드 제어."""
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException

from .control import TeleopState


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit("대화형 터미널이 필요합니다. Windows에서 ssh -t로 접속하세요.")
    rclpy.init(args=args)
    node = rclpy.create_node("robot_teleop")
    node.declare_parameter("key_timeout_s", 1.0)
    key_timeout = float(node.get_parameter("key_timeout_s").value)
    if not 0.1 <= key_timeout <= 5.0:
        raise ValueError("key_timeout_s는 0.1~5.0초여야 합니다.")
    pub = node.create_publisher(Twist, "cmd_vel", 1)
    state = TeleopState()
    settings = termios.tcgetattr(sys.stdin)
    last_key = time.monotonic()
    next_publish = last_key
    print("w/x: 선속도 ±0.01m/s, a/d: 각속도 ±0.1rad/s")
    print("s 또는 Space: 즉시 정지, Ctrl+C: 종료")
    print(f"상한: 선속도 0.2m/s, 각속도 0.5rad/s. {key_timeout:g}초간 조작이 없으면 정지합니다.")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            timeout = max(0.0, min(0.1, next_publish - time.monotonic()))
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if ready:
                key = sys.stdin.read(1)
                if key in ("", "\x03"):
                    break
                if key in "wxads ":
                    state.key(key)
                    last_key = time.monotonic()
                    print(f"\r목표 v={state.linear:+.2f}m/s, w={state.angular:+.1f}rad/s  ",
                          end="", flush=True)
                    if key in ("s", " "):
                        pub.publish(Twist())
            now = time.monotonic()
            if now - last_key >= key_timeout:
                state.stop()
            if now >= next_publish:
                linear, angular = state.step()
                message = Twist()
                message.linear.x = linear
                message.angular.z = angular
                pub.publish(message)
                next_publish = now + 0.1
            rclpy.spin_once(node, timeout_sec=0.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.0)
        finally:
            # ROS context가 먼저 종료되어도 SSH 터미널 설정은 복원합니다.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
