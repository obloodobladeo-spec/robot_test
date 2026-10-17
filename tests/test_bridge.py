"""ROS 및 USB 경계만 대체하여 실제 브리지의 정지·매핑 경로를 검사합니다."""
import importlib
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson/robot_ws/src/robot_test"))


class Message:
    def __init__(self, data=None):
        self.data = data
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class SerialError(OSError):
    pass


class FakePort:
    def __init__(self):
        self.written = []
        self.incoming = bytearray()
        self.is_open = True
        self.fail = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def reset_input_buffer(self):
        self.incoming.clear()

    def write(self, data):
        if self.fail:
            raise SerialError("USB removed")
        self.written.append(data)
        return len(data)

    def read(self, count):
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def close(self):
        self.is_open = False


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.params = {}

    def declare_parameter(self, name, value):
        self.params[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda message: None)

    def create_subscription(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None, error=lambda message: None)


def module(name, **attributes):
    result = ModuleType(name)
    result.__dict__.update(attributes)
    return result


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.port = FakePort()
        stubs = {
            "rclpy": module("rclpy"),
            "rclpy.node": module("rclpy.node", Node=FakeNode),
            "rclpy.executors": module("rclpy.executors", ExternalShutdownException=RuntimeError),
            "rclpy.qos": module("rclpy.qos", QoSProfile=lambda **kw: None,
                DurabilityPolicy=SimpleNamespace(VOLATILE=0),
                ReliabilityPolicy=SimpleNamespace(RELIABLE=0)),
            "serial": module("serial", Serial=lambda *args, **kw: self.port,
                             SerialException=SerialError),
            "geometry_msgs": module("geometry_msgs"),
            "geometry_msgs.msg": module("geometry_msgs.msg", Twist=Message),
            "std_msgs": module("std_msgs"),
            "std_msgs.msg": module("std_msgs.msg", Float64MultiArray=Message, String=Message),
        }
        self.modules = patch.dict(sys.modules, stubs)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        sys.modules.pop("robot_test.serial_bridge", None)
        self.bridge_module = importlib.import_module("robot_test.serial_bridge")
        self.clock = patch.object(self.bridge_module.time, "monotonic", return_value=10.0)
        self.clock_mock = self.clock.start()
        self.addCleanup(self.clock.stop)
        FakeNode.overrides = {"enable_motors": True, "wheel_separation_m": 0.4,
                              "motor1_side": "left"}

    def build(self):
        node = self.bridge_module.SerialBridge()
        self.addCleanup(node.close)
        return node

    def feedback(self, node):
        from robot_test.protocol import frame
        node.consume_feedback(frame("S,1,0,0,0,0,0"))

    def cmd(self, node, linear=0.1, angular=0.0):
        message = Message()
        message.linear.x, message.angular.z = linear, angular
        node.on_cmd(message)

    def targets(self):
        parts = self.port.written[-1].split(b"*")[0].split(b",")
        return int(parts[2]), int(parts[3])

    def test_valid_command_and_left_turn(self):
        node = self.build()
        self.feedback(node)
        self.cmd(node, 0.0, 0.5)
        node.tick()
        self.assertEqual(self.targets(), (-100, 100))

    def test_motor1_on_right_swaps_mapping(self):
        FakeNode.overrides["motor1_side"] = "right"
        node = self.build()
        self.feedback(node)
        self.cmd(node, 0.0, 0.5)
        node.tick()
        self.assertEqual(self.targets(), (100, -100))

    def test_default_lock_never_sends_motion(self):
        FakeNode.overrides = {}
        node = self.build()
        self.feedback(node)
        self.cmd(node)
        node.tick()
        self.assertEqual(self.targets(), (0, 0))

    def test_enabled_requires_real_geometry(self):
        FakeNode.overrides["wheel_separation_m"] = 0.0
        with self.assertRaises(ValueError):
            self.build()

    def test_stale_cmd_stops_with_fresh_feedback(self):
        node = self.build()
        self.feedback(node)
        self.cmd(node)
        self.clock_mock.return_value = 10.31
        self.feedback(node)
        node.tick()
        self.assertEqual(self.targets(), (0, 0))

    def test_feedback_loss_clears_previous_command(self):
        node = self.build()
        self.feedback(node)
        self.clock_mock.return_value = 10.6
        self.cmd(node)
        node.tick()
        self.assertEqual(self.targets(), (0, 0))
        self.feedback(node)
        node.tick()
        self.assertEqual(self.targets(), (0, 0))
        self.cmd(node)
        node.tick()
        self.assertEqual(self.targets(), (100, 100))

    def test_nan_and_corrupt_feedback_do_not_enable_motion(self):
        node = self.build()
        node.consume_feedback(b"S,1,0,0,0,0,0*0000\n")
        self.cmd(node)
        node.tick()
        self.assertEqual(self.targets(), (0, 0))
        self.feedback(node)
        self.cmd(node, float("nan"))
        node.tick()
        self.assertEqual(self.targets(), (0, 0))

    def test_usb_failure_latches_closed(self):
        node = self.build()
        self.feedback(node)
        self.cmd(node)
        self.port.fail = True
        node.tick()
        self.assertTrue(node.link_failed)
        self.assertFalse(self.port.is_open)

    def test_oversize_line_resynchronizes_on_newline(self):
        node = self.build()
        node.consume_feedback(b"x" * 200 + b"\n")
        self.assertEqual(node.last_feedback, float("-inf"))
        self.feedback(node)
        self.assertEqual(node.last_feedback, 10.0)


if __name__ == "__main__":
    unittest.main()
