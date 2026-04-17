#!/usr/bin/env python3
"""Phase-1 QDrone 2 mission server.

Speaks QUARC stream protocol on TCP 0.0.0.0:18001 against the pre-compiled
QD2_DroneStack_PID_2021a binary. DroneStack dials us (it's the TCP client);
we are the server.

Per-tick (~250 Hz) packet layout (little-endian float64):

    server -> drone  (128 bytes, 16 doubles):
        [0..3]   triggers[4]        [arm, takeoff, estop, joystick_issue]
        [4]      is_tracking        1 if mocap pose valid (phase 1: 0)
        [5..10]  measured_pose[6]   [x, y, z, roll, pitch, yaw]  (phase 1: zeros)
        [11..14] desired_pose[4]    [x, y, z, yaw]
        [15]     timestamp          seconds

    drone -> server  (8 bytes, 1 double):
        [0]      timestamp echo     drone loops back [15] of last rx packet

Phase 1 is an autonomous hardcoded sequence:
    IDLE -> ARM-HOLD -> HOVER (z=altitude) -> LAND -> DISARM -> exit
with is_tracking=0, measured_pose=0, desired_pose=(0,0,altitude,0).

Running this server alone does NOT spin motors. The drone's stabilizer
binary must be launched separately via `drone.py hw run QD2_DroneStack_PID_2021a`.
If DroneStack is not running, this script binds the port and waits until
--connect-timeout (default 30 s).
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

try:
    from pal.utilities.stream import BasicStream
    from quanser.common import Timeout
except ImportError as e:
    print(f"[mission] missing Quanser python API: {e}", flush=True)
    sys.exit(2)


ROOT = Path(__file__).resolve().parent.parent
PID_DIR = ROOT / ".pids"
PID_FILE = PID_DIR / "mission.pid"

TICK_HZ = 250.0
TICK_DT = 1.0 / TICK_HZ
PACKET_FLOATS = 16


def build_packet(arm, takeoff, is_tracking, desired_z, timestamp):
    pkt = np.zeros(PACKET_FLOATS, dtype=np.float64)
    pkt[0] = float(arm)
    pkt[1] = float(takeoff)
    # pkt[2] estop = 0, pkt[3] joystick_issue = 0
    pkt[4] = float(is_tracking)
    # pkt[5..10] measured_pose = 0
    # pkt[11..13] desired_pose x,y = 0
    pkt[13] = float(desired_z)
    # pkt[14] desired yaw = 0
    pkt[15] = float(timestamp)
    return pkt


def log(msg):
    print(f"[mission {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _cleanup(server):
    try:
        server.terminate()
    except Exception:
        pass
    try:
        PID_FILE.unlink()
    except FileNotFoundError:
        pass


def main():
    ap = argparse.ArgumentParser(description="QDrone 2 phase-1 mission server")
    ap.add_argument("--port", type=int, default=18001)
    ap.add_argument("--altitude", type=float, default=0.5,
                    help="hover altitude in meters (default 0.5)")
    ap.add_argument("--hover", type=float, default=5.0,
                    help="hover duration in seconds (default 5.0)")
    ap.add_argument("--preflight", type=float, default=3.0,
                    help="idle seconds before arm (all zeros sent)")
    ap.add_argument("--arm-hold", type=float, default=3.0,
                    help="seconds with arm=1 takeoff=0 before takeoff")
    ap.add_argument("--land-hold", type=float, default=5.0,
                    help="seconds with arm=1 takeoff=0 for descent")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    args = ap.parse_args()

    PID_DIR.mkdir(exist_ok=True)
    PID_FILE.write_text(str(os.getpid()))

    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT,  lambda *_: stop.__setitem__(0, True))

    uri = f"tcpip://0.0.0.0:{args.port}"
    recv_buf = np.zeros(1, dtype=np.float64)
    server = BasicStream(uri=uri,
                         agent="S",
                         receiveBuffer=recv_buf,
                         sendBufferSize=2048,
                         recvBufferSize=2048,
                         nonBlocking=True)

    log(f"listening on {uri}")
    log(f"plan: idle {args.preflight:.1f}s -> arm {args.arm_hold:.1f}s -> "
        f"hover {args.hover:.1f}s @ z={args.altitude}m -> land {args.land_hold:.1f}s -> disarm")

    poll_to = Timeout(seconds=0, nanoseconds=1_000_000)
    connect_deadline = time.monotonic() + args.connect_timeout
    while not server.connected and not stop[0]:
        server.checkConnection(timeout=poll_to)
        if server.connected:
            break
        if time.monotonic() > connect_deadline:
            log("no client connected within timeout, exiting")
            _cleanup(server)
            return 1
        time.sleep(0.05)

    if stop[0]:
        log("stop signal before connection; exiting clean")
        _cleanup(server)
        return 0

    log("drone connected, entering sequence")

    t_preflight_end = args.preflight
    t_arm_end       = t_preflight_end + args.arm_hold
    t_hover_end     = t_arm_end + args.hover
    t_land_end      = t_hover_end + args.land_hold
    t_total         = t_land_end + 1.0

    t0 = time.monotonic()
    next_tick = t0
    last_status = 0.0
    last_phase = None

    while not stop[0]:
        now = time.monotonic()
        elapsed = now - t0

        if elapsed < t_preflight_end:
            phase, arm, takeoff, dz = "IDLE",   0, 0, 0.0
        elif elapsed < t_arm_end:
            phase, arm, takeoff, dz = "ARM",    1, 0, 0.0
        elif elapsed < t_hover_end:
            phase, arm, takeoff, dz = "HOVER",  1, 1, args.altitude
        elif elapsed < t_land_end:
            phase, arm, takeoff, dz = "LAND",   1, 0, 0.0
        elif elapsed < t_total:
            phase, arm, takeoff, dz = "DISARM", 0, 0, 0.0
        else:
            log("sequence complete")
            break

        if phase != last_phase:
            log(f"-> {phase}")
            last_phase = phase

        pkt = build_packet(arm, takeoff, 0, dz, time.time())
        try:
            server.send(pkt)
            server.receive(timeout=poll_to, iterations=1)
        except Exception as e:
            log(f"stream err (phase={phase}): {e}")

        if now - last_status >= 1.0:
            last_status = now
            log(f"phase={phase} t={elapsed:5.2f}s  arm={arm} takeoff={takeoff} "
                f"dz={dz:.2f}  rx_ts={recv_buf[0]:.3f}")

        next_tick += TICK_DT
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()

    log("exiting, sending disarm tail")
    zeros = build_packet(0, 0, 0, 0.0, time.time())
    for _ in range(50):
        try:
            server.send(zeros)
        except Exception:
            pass
        time.sleep(TICK_DT)

    _cleanup(server)
    return 0


if __name__ == "__main__":
    sys.exit(main())
