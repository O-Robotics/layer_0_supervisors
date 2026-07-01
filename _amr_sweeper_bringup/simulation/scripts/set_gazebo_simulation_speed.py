#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time


def _run(speed: float, world: str) -> int:
    command = [
        "gz",
        "service",
        "-s",
        f"/world/{world}/set_physics",
        "--reqtype",
        "gz.msgs.Physics",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        f"max_step_size: 0.001 real_time_factor: {speed} real_time_update_rate: 1000",
    ]

    deadline = time.time() + 60.0
    completed = None
    while time.time() < deadline:
        completed = subprocess.run(command, capture_output=True, text=True)
        if completed.returncode == 0:
            if completed.stdout.strip():
                print(completed.stdout.strip())
            return 0
        time.sleep(1.0)

    if completed is not None:
        if completed.stderr.strip():
            print(completed.stderr.strip(), file=sys.stderr)
        elif completed.stdout.strip():
            print(completed.stdout.strip(), file=sys.stderr)
        return completed.returncode or 1
    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--world", default="amr_sweeper_test")
    args = parser.parse_args()
    raise SystemExit(_run(args.speed, args.world))
