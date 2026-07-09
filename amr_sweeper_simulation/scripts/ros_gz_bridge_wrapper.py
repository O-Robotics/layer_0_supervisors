#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import threading
import time

_SHUTDOWN_TIMEOUT_SEC = 2.0
_shutdown_requested = False
_child_process = None
_shutdown_lock = threading.Lock()


def _terminate_bridge_process_group(sig: int) -> None:
    child = _child_process
    if child is None or child.poll() is not None:
        return

    try:
        os.killpg(child.pid, sig)
    except ProcessLookupError:
        pass


def _escalate_shutdown_after_timeout() -> None:
    deadline = time.monotonic() + _SHUTDOWN_TIMEOUT_SEC
    while time.monotonic() < deadline:
        child = _child_process
        if child is None or child.poll() is not None:
            return
        time.sleep(0.05)

    _terminate_bridge_process_group(signal.SIGKILL)


def _request_shutdown(signum, _frame) -> None:
    global _shutdown_requested
    with _shutdown_lock:
        if _shutdown_requested:
            child = _child_process
            if child is not None and child.poll() is None:
                _terminate_bridge_process_group(signal.SIGKILL)
            return

        _shutdown_requested = True
        child = _child_process
        if child is not None and child.poll() is None:
            _terminate_bridge_process_group(signal.SIGTERM)
            threading.Thread(target=_escalate_shutdown_after_timeout, daemon=True).start()


def main() -> int:
    global _child_process

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    command = [
        'ros2',
        'run',
        'ros_gz_bridge',
        'bridge_node',
        *sys.argv[1:],
    ]

    _child_process = subprocess.Popen(
        command,
        env=os.environ.copy(),
        start_new_session=True,
    )
    return_code = _child_process.wait()

    if _shutdown_requested:
        return 0
    return return_code


if __name__ == '__main__':
    sys.exit(main())