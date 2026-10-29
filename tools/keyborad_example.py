from unittest.mock import patch

import learning_serial


class TestPort:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def write(self, data):
        print("전송 확인:", data.decode("ascii").strip())
        return len(data)

    def read(self, size):
        # 실제 STM32가 없으므로 수신 데이터는 없습니다.
        return b""


if __name__ == "__main__":
    # 이 시험 중에만 실제 포트를 시험용 포트로 대체합니다.
    with patch(
        "learning_serial.serial.Serial",
        return_value=TestPort(),
    ):
        learning_serial.main()