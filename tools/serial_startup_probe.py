"""Compare serial startup buffering on Windows and WSL without moving motors."""

import argparse
import json
import math
from pathlib import Path
import platform
import time

import serial


STOP_COMMAND = b"V=0 W=0\n"


def run_trial(port_name: str, duration: float) -> dict:
    events = []
    pending = bytearray()
    with serial.Serial(
        port=port_name,
        baudrate=115200,
        timeout=0,
        write_timeout=0.1,
    ) as port:
        before_reset = port.in_waiting
        port.reset_input_buffer()
        after_reset = port.in_waiting
        started = time.monotonic()
        next_tick = started
        try:
            while time.monotonic() - started < duration:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(0.005, next_tick - now))
                    continue

                if port.write(STOP_COMMAND) != len(STOP_COMMAND):
                    raise OSError("Incomplete stop-command write")
                chunk = port.read(256)
                received_at = time.monotonic() - started
                pending.extend(chunk)
                while b"\n" in pending:
                    line, _, remaining = pending.partition(b"\n")
                    pending = bytearray(remaining)
                    events.append({
                        "elapsed_s": received_at,
                        "line": line.decode("ascii", errors="replace").strip(),
                    })
                if len(pending) > 4096:
                    raise ValueError("Serial input has no line terminator")
                # Match the bridge's 100 ms polling and 256-byte read limit.
                next_tick = time.monotonic() + 0.1
        finally:
            port.write(STOP_COMMAND)
            port.flush()

    intervals = [
        current["elapsed_s"] - previous["elapsed_s"]
        for previous, current in zip(events, events[1:])
    ]
    return {
        "port": port_name,
        "duration_s": duration,
        "bytes_before_reset": before_reset,
        "bytes_after_reset": after_reset,
        "line_count": len(events),
        "lines_in_first_second": sum(e["elapsed_s"] < 1 for e in events),
        "intervals_s": intervals,
        "events": events,
        "partial_tail": pending.decode("ascii", errors="replace"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=6.5)
    parser.add_argument("--trials", type=int, default=2)
    parser.add_argument("--idle", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    if not math.isfinite(args.idle) or args.idle < 0 or args.trials < 1:
        parser.error("idle must be finite and nonnegative; trials must be positive")

    result = {"platform": platform.platform(), "trials": []}
    try:
        for number in range(1, args.trials + 1):
            print(f"Trial {number}: port closed for {args.idle:g}s", flush=True)
            time.sleep(args.idle)
            trial = run_trial(args.port, args.duration)
            result["trials"].append(trial)
            summary = {k: v for k, v in trial.items() if k not in ("events", "intervals_s")}
            summary["line_offsets_s"] = [round(e["elapsed_s"], 4) for e in trial["events"]]
            print(json.dumps(summary), flush=True)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"Saved: {args.output}", flush=True)


if __name__ == "__main__":
    main()
