import time
import serial
import csv
import msvcrt

def parse_feedback(text):
    values ={}

    for item in text.split():
        key, value = item.split("=")
        values[key] = int(value)

    return values

PORT = "COM11"
BAUDRATE = 115200

def main():
    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0,
        write_timeout=0.1,
    ) as stm, open(
        "motor_log.csv",
        "a",
        newline="",
        encoding="utf-8",
    ) as log_file:

        writer = csv.writer(log_file)

        if log_file.tell() == 0:
            writer.writerow([
                "elapsed_s",
                "target_linear_mm_s",
                "target_angular_mrad_s",
                "left_speed_mm_s",
                "right_speed_mm_s",
                "x_mm",
                "y_mm",
            ])

        start_time = time.monotonic()

        linear_speed = 0
        angular_speed = 0

        previous_time = time.monotonic()
        last_keytime = previous_time

        key_timeout_s = 1.0

        rx_buffer = bytearray()

        try:
            while(True):

                if msvcrt.kbhit():
                    key = msvcrt.getwch().lower()

                    if key in ("w", "x", "s", "a", "d"):
                        last_keytime = time.monotonic()

                    if key == "w":
                        linear_speed = min(linear_speed + 10, 200)

                    elif key == "x":
                        linear_speed = max(linear_speed - 10, -200)

                    elif key == "a":
                        angular_speed = min(angular_speed + 100, 500)

                    elif key == "d":
                        angular_speed = max(angular_speed - 100, -500)

                    elif key == "s":
                        linear_speed = 0
                        angular_speed = 0

                    elif key == "q":
                        break

                now = time.monotonic()

                if (now - last_keytime ) >= key_timeout_s:
                    linear_speed = 0
                    angular_speed = 0

                if (now - previous_time) >= 0.1:
                    previous_time = now

                    command = (
                        f"V={linear_speed} W={angular_speed}\n"
                    )

                    stm.write(command.encode("ascii"))
                    print(command.strip())

                received = stm.read(256)
                rx_buffer.extend(received)

                while b"\n" in rx_buffer:
                    line, _, remaining = rx_buffer.partition(b"\n")
                    rx_buffer = bytearray(remaining)

                    text = line.decode(
                        "ascii",
                        errors="replace",
                    ).strip()

                    try:
                        feedback = parse_feedback(text)

                        left_speed = feedback["L"]
                        right_speed = feedback["R"]
                        x_mm = feedback["X"]
                        y_mm = feedback["Y"]

                    except (ValueError, KeyError):
                        print("해석 할 수 없는 응답 :", text)
                        continue

                    print(
                        f"left speed: {left_speed}",
                        f"right speed: {right_speed}",
                        f"X= {x_mm} mm, Y= {y_mm} mm"
                    )

                    elapsed_s = time.monotonic() - start_time

                    writer.writerow([
                        round(elapsed_s, 3),
                        linear_speed,
                        angular_speed,
                        left_speed,
                        right_speed,
                        x_mm,
                        y_mm,
                    ])
                    log_file.flush()

                time.sleep(0.001)

        except KeyboardInterrupt:
            print("프로그램을 종료합니다.")

        finally:
            stm.write(b"V=0 W=0\n")

if __name__=="__main__":
    main()