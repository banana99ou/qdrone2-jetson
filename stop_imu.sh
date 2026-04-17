#!/bin/bash
# Stops ONLY the qdrone imu_service.py process tracked in .pids/imu.pid.
# Verifies the target is actually imu_service.py before signalling, so a
# stale pid file can't be used to kill arbitrary root processes.
set -euo pipefail

PID_FILE="/home/nvidia/ros2/qdrone/.pids/imu.pid"
EXPECT_CMD="imu_service.py"

if [ ! -f "$PID_FILE" ]; then
    echo "[stop_imu] no pid file at $PID_FILE; nothing to do"
    exit 0
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
    echo "[stop_imu] pid file content is not a number: '$pid'"
    exit 1
fi

if ! kill -0 "$pid" 2>/dev/null; then
    echo "[stop_imu] pid $pid not running; cleaning pid file"
    rm -f "$PID_FILE"
    exit 0
fi

# Verify the target is actually our imu_service.py before sending any signal.
# Reading /proc/<pid>/cmdline is a string with NUL separators.
cmdline="$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null || true)"
if ! [[ "$cmdline" == *"$EXPECT_CMD"* ]]; then
    echo "[stop_imu] pid $pid is NOT $EXPECT_CMD (cmdline=$cmdline); refusing to kill"
    exit 2
fi

echo "[stop_imu] sending SIGTERM to pid $pid"
kill -TERM "$pid"

for _ in $(seq 1 30); do
    sleep 0.1
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[stop_imu] stopped"
        exit 0
    fi
done

echo "[stop_imu] still running; escalating to SIGKILL"
kill -KILL "$pid"
