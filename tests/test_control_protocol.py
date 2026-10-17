"""ROS·모터 없이 실행하는 변환·제한·통신 회귀 검사."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson/robot_ws/src/robot_test"))
from robot_test.control import TeleopState, wheel_targets
from robot_test.protocol import command, crc16, frame, parse_status


class ControlTests(unittest.TestCase):
    def test_straight_and_reverse(self):
        self.assertEqual(wheel_targets(0.1, 0.0, 0.3), (100, 100))
        self.assertEqual(wheel_targets(-0.1, 0.0, 0.3), (-100, -100))

    def test_positive_yaw_turns_left(self):
        self.assertEqual(wheel_targets(0.0, 0.4, 0.3), (-60, 60))

    def test_combined_velocity_limits_both_wheels(self):
        self.assertEqual(wheel_targets(0.2, 0.5, 0.4), (67, 200))
        for v in (-0.2, 0, 0.2):
            for w in (-0.5, 0, 0.5):
                self.assertLessEqual(max(map(abs, wheel_targets(v, w, 0.4))), 200)

    def test_invalid_geometry_or_nonfinite_input(self):
        for args in [(0.1, 0, 0), (0.1, 0, -1), (float("nan"), 0, 0.3),
                     (0, float("inf"), 0.3), (0.1, 0, 0.3, 0.21)]:
            with self.assertRaises(ValueError):
                wheel_targets(*args)

    def test_turtlebot_key_steps_and_stop(self):
        state = TeleopState()
        state.key("w")
        state.key("a")
        self.assertAlmostEqual(state.linear, 0.01)
        self.assertAlmostEqual(state.angular, 0.1)
        self.assertEqual(state.step(), (0.005, 0.05))
        state.key("s")
        self.assertEqual(state.step(), (0.0, 0.0))

    def test_teleop_limits(self):
        state = TeleopState()
        for _ in range(200):
            state.key("w")
            state.key("a")
        self.assertEqual((state.linear, state.angular), (0.2, 0.5))


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_command_negative_and_bounds(self):
        self.assertTrue(command(65535, -200, 200).startswith(b"V,65535,-200,200*"))
        for args in [(0, 201, 0), (0, 0, -201), (-1, 0, 0)]:
            with self.assertRaises(ValueError):
                command(*args)

    def test_status_roundtrip(self):
        status = parse_status(frame("S,19,-100,101,1200,1400,0"))
        self.assertEqual((status.sequence, status.motor1_mm_s, status.motor2_mm_s), (19, -100, 101))
        self.assertFalse(status.timed_out)

    def test_corruption_and_truncation(self):
        raw = frame("S,1,0,0,0,0,1")
        for bad in [raw.replace(b"S,1", b"S,2"), raw[:-4], b"noise", b"x" * 96]:
            with self.assertRaises(ValueError):
                parse_status(bad)

    def test_status_range(self):
        for payload in ["S,1,0,0,5001,0,0", "S,1,0,0,0,5001,0",
                        "S,65536,0,0,0,0,0", "S,1,0,0,0,0,2"]:
            with self.assertRaises(ValueError):
                parse_status(frame(payload))

    def test_status_accepts_fifty_percent_duty(self):
        status = parse_status(frame("S,1,200,-200,5000,5000,0"))
        self.assertEqual((status.motor1_duty_centi, status.motor2_duty_centi), (5000, 5000))


if __name__ == "__main__":
    unittest.main()
