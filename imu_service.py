#!/usr/bin/env python3
"""QDrone 2 IMU → pose publisher.

Opens the Quanser HIL card, reads gyro (ch 3000-3002, rad/s) and accel
(ch 4000-4002, m/s^2) at SCAN_RATE Hz, runs a complementary filter for
attitude, and publishes the pose over UDP to 127.0.0.1:8090 at PUB_RATE Hz.

Position is NOT estimated here — IMU-only dead reckoning drifts meters in
seconds. Hook up optical flow, VIO, or external mocap later for position.

Must run as root (HIL hardware access is privileged):
    sudo python3 imu_service.py

UDP packet layout (little-endian, 36 bytes):
    double  host_ts_unix
    float32 qw, qx, qy, qz      (body->world quaternion, wxyz)
    float32 px, py, pz          (world position, meters; currently 0)
"""
import math
import os
import signal
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
PID_DIR = ROOT / ".pids"
PID_FILE = PID_DIR / "imu.pid"

SCAN_RATE = 200.0                  # HIL read rate [Hz]
PUB_RATE = 100.0                   # UDP publish rate [Hz]
UDP_ADDR = ("127.0.0.1", 8090)
ACCEL_BETA = 0.02                  # tilt-correction gain (per filter step)


def qmul(a, b):
    return np.array([
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ])


def qnorm(q):
    n = math.sqrt(float(q @ q))
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def qrot_inv(q, v):
    qi = np.array([q[0], -q[1], -q[2], -q[3]])
    return qmul(qmul(qi, np.array([0.0, v[0], v[1], v[2]])), q)[1:]


def _unlink_pid():
    try:
        PID_FILE.unlink()
    except FileNotFoundError:
        pass


def main():
    try:
        from quanser.hardware import HIL
    except ImportError as e:
        print(f"[imu] missing quanser python API: {e}", flush=True)
        return 2

    PID_DIR.mkdir(exist_ok=True)
    PID_FILE.write_text(str(os.getpid()))

    stop_flag = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.__setitem__(0, True))
    signal.signal(signal.SIGINT,  lambda *_: stop_flag.__setitem__(0, True))

    print(f"[imu] pid={os.getpid()} uid={os.getuid()}", flush=True)

    try:
        card = HIL("qdrone2", "0")
    except Exception as e:
        print(f"[imu] HIL open failed: {e}", flush=True)
        _unlink_pid()
        return 1
    print("[imu] HIL opened", flush=True)

    other_ch = np.array([3000, 3001, 3002, 4000, 4001, 4002], dtype=np.uint32)
    buf = np.zeros(6, dtype=np.float64)

    card.read_other(other_ch, len(other_ch), buf)
    ax, ay, az = buf[3], buf[4], buf[5]
    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    q = qnorm(np.array([cr*cp, sr*cp, cr*sp, -sr*sp]))
    print(f"[imu] init roll={math.degrees(roll):.1f}deg pitch={math.degrees(pitch):.1f}deg", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / SCAN_RATE
    pub_period = 1.0 / PUB_RATE
    next_tick = time.monotonic()
    last_pub = 0.0
    err_streak = 0

    while not stop_flag[0]:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        next_tick += period
        dt = period

        try:
            card.read_other(other_ch, len(other_ch), buf)
        except Exception as e:
            err_streak += 1
            if err_streak % 50 == 1:
                print(f"[imu] read err (#{err_streak}): {e}", flush=True)
            time.sleep(0.1)
            continue
        err_streak = 0

        gx, gy, gz = buf[0], buf[1], buf[2]
        ax, ay, az = buf[3], buf[4], buf[5]

        # gyro integration (omega expressed as pure quat)
        dq = 0.5 * qmul(q, np.array([0.0, gx, gy, gz])) * dt
        q = qnorm(q + dq)

        # accelerometer tilt correction (skip during accel transients)
        a_mag = math.sqrt(ax*ax + ay*ay + az*az)
        if 7.0 < a_mag < 12.0:
            g_body = qrot_inv(q, np.array([0.0, 0.0, -1.0]))
            a_hat = np.array([ax, ay, az]) / a_mag
            err = np.cross(g_body, a_hat)
            w_corr = np.array([0.0, ACCEL_BETA*err[0], ACCEL_BETA*err[1], ACCEL_BETA*err[2]])
            q = qnorm(q + 0.5 * qmul(q, w_corr))

        if now - last_pub >= pub_period:
            last_pub = now
            pkt = struct.pack("<d4f3f",
                              time.time(),
                              float(q[0]), float(q[1]), float(q[2]), float(q[3]),
                              0.0, 0.0, 0.0)
            try:
                sock.sendto(pkt, UDP_ADDR)
            except OSError:
                pass

    print("[imu] exiting", flush=True)
    try: card.close()
    except Exception: pass
    _unlink_pid()
    return 0


if __name__ == "__main__":
    sys.exit(main())
