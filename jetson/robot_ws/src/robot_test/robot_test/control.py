"""ROS와 하드웨어에 의존하지 않는 단위 변환·teleop 계산."""
from dataclasses import dataclass
import math


def clamp(value: float, limit: float) -> float:
    if not math.isfinite(value) or not math.isfinite(limit) or limit < 0:
        raise ValueError("유한한 값과 음수가 아닌 제한값이 필요합니다.")
    return max(-limit, min(limit, value))


def wheel_targets(linear: float, angular: float, separation: float,
                  max_wheel_speed: float = 0.2) -> tuple[int, int]:
    """ROS 차체 속도(m/s, rad/s)를 좌우 바퀴 목표(mm/s)로 변환합니다."""
    if not all(math.isfinite(x) for x in (linear, angular, separation, max_wheel_speed)):
        raise ValueError("속도와 바퀴 간격은 유한한 값이어야 합니다.")
    if separation <= 0 or not 0 < max_wheel_speed <= 0.2:
        raise ValueError("바퀴 간격은 양수, 바퀴 속도 상한은 0~0.2m/s여야 합니다.")
    left = linear - angular * separation / 2.0
    right = linear + angular * separation / 2.0
    # 한 바퀴만 잘라 회전 곡률을 바꾸지 않고 두 바퀴를 같은 비율로 제한합니다.
    scale = max(1.0, abs(left) / max_wheel_speed, abs(right) / max_wheel_speed)
    limit_mm_s = min(200, int(max_wheel_speed * 1000))
    return (max(-limit_mm_s, min(limit_mm_s, round(left / scale * 1000))),
            max(-limit_mm_s, min(limit_mm_s, round(right / scale * 1000))))


def approach(current: float, target: float, step: float) -> float:
    return min(target, current + step) if target > current else max(target, current - step)


@dataclass
class TeleopState:
    linear: float = 0.0
    angular: float = 0.0
    output_linear: float = 0.0
    output_angular: float = 0.0

    def stop(self) -> None:
        self.linear = self.angular = self.output_linear = self.output_angular = 0.0

    def key(self, key: str) -> None:
        # TurtleBot3의 키 배치와 증분을 참고한 독립 구현입니다.
        if key == "w":
            self.linear = clamp(self.linear + 0.01, 0.2)
        elif key == "x":
            self.linear = clamp(self.linear - 0.01, 0.2)
        elif key == "a":
            self.angular = clamp(self.angular + 0.1, 0.5)
        elif key == "d":
            self.angular = clamp(self.angular - 0.1, 0.5)
        elif key in ("s", " "):
            self.stop()

    def step(self) -> tuple[float, float]:
        self.output_linear = approach(self.output_linear, self.linear, 0.005)
        self.output_angular = approach(self.output_angular, self.angular, 0.05)
        return self.output_linear, self.output_angular
