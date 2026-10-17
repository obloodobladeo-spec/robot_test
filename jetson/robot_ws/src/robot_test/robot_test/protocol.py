"""115200 8N1 ASCII 프레임. CRC-16/CCITT-FALSE, 초기값 0xFFFF."""
from dataclasses import dataclass
import re


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(payload: str) -> bytes:
    raw = payload.encode("ascii")
    if any(value < 32 or value > 126 for value in raw) or b"*" in raw:
        raise ValueError("프레임에 허용되지 않는 문자가 있습니다.")
    return raw + f"*{crc16(raw):04X}\n".encode("ascii")


def command(sequence: int, motor1: int, motor2: int) -> bytes:
    if not 0 <= sequence <= 65535 or not all(-200 <= x <= 200 for x in (motor1, motor2)):
        raise ValueError("명령 범위를 벗어났습니다.")
    return frame(f"V,{sequence},{motor1},{motor2}")


@dataclass(frozen=True)
class Status:
    sequence: int
    motor1_mm_s: int
    motor2_mm_s: int
    motor1_duty_centi: int
    motor2_duty_centi: int
    timed_out: bool


def parse_status(data: bytes) -> Status:
    if len(data) > 95:
        raise ValueError("프레임 길이 초과")
    raw = data.rstrip(b"\r\n")
    try:
        payload, checksum = raw.split(b"*")
    except ValueError as exc:
        raise ValueError("프레임 형식 오류") from exc
    if not re.fullmatch(rb"[0-9A-Fa-f]{4}", checksum) or crc16(payload) != int(checksum, 16):
        raise ValueError("CRC 오류")
    if not re.fullmatch(rb"S,\d+,-?\d+,-?\d+,\d+,\d+,[01]", payload):
        raise ValueError("상태 형식 오류")
    seq, m1, m2, d1, d2, timeout = map(int, payload.decode("ascii").split(",")[1:])
    if not 0 <= seq <= 65535 or not all(0 <= x <= 5000 for x in (d1, d2)):
        raise ValueError("상태 범위 오류")
    return Status(seq, m1, m2, d1, d2, bool(timeout))
