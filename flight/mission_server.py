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

Uses quanser.communications.Stream directly; pal.utilities.stream is not
installed on this drone image.
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

try:
    from quanser.communications import Stream, PollFlag
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


def log(msg):
    print(f"[mission {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _unlink(path):
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def build_packet(arm, takeoff, is_tracking, desired_z, timestamp, buf):
    """Fill a 16-element float64 buffer matching DroneStack's expected layout.

    Layout (16 float64):
      [0..3]   triggers[4]        [arm, takeoff, estop, joystick_issue]
      [4]      is_tracking
      [5..10]  measured_pose[6]   [x, y, z, roll, pitch, yaw] (m, rad)
      [11..14] desired_pose[4]    [x, y, z, yaw] (m, rad)
      [15]     timestamp          seconds
    """
    buf[:] = 0.0
    buf[0]  = float(arm)
    buf[1]  = float(takeoff)
    # buf[2] estop = 0, buf[3] joystick_issue = 0
    buf[4]  = float(is_tracking)
    # measured_pose fixed at origin + level (no real pose source in phase 1):
    # buf[5..10] = 0 means x=y=z=0, roll=pitch=yaw=0 → "I'm at origin, level"
    # desired_pose: x=y=yaw=0, z = desired_z
    buf[13] = float(desired_z)
    buf[15] = float(timestamp)


def wait_for_client(server, stop, deadline):
    """Poll for an incoming connection until deadline or stop. Return accepted
    client Stream, or None on timeout/stop."""
    poll_to = Timeout(seconds=0, nanoseconds=10_000_000)  # 10 ms
    while not stop[0]:
        try:
            # poll() returns the subset of requested flags that are ready
            ready = server.poll(poll_to, PollFlag.ACCEPT)
            if ready & PollFlag.ACCEPT:
                client = server.accept(2048, 2048)
                if client is not None:
                    return client
        except Exception as e:
            # non-fatal; typically WOULD_BLOCK variants in non-blocking mode
            pass
        if time.monotonic() > deadline:
            return None
        time.sleep(0.01)
    return None


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
    ap.add_argument("--idle-only", action="store_true",
                    help="handshake test: stay in IDLE forever, never arm. "
                         "Motors will not spin. Ctrl-C to exit.")
    args = ap.parse_args()

    PID_DIR.mkdir(exist_ok=True)
    PID_FILE.write_text(str(os.getpid()))

    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT,  lambda *_: stop.__setitem__(0, True))

    uri = f"tcpip://0.0.0.0:{args.port}"
    server = Stream()
    try:
        server.listen(uri, True)  # non-blocking
    except Exception as e:
        log(f"listen on {uri} failed: {e}")
        _unlink(PID_FILE)
        return 1

    log(f"listening on {uri}")
    log(f"plan: idle {args.preflight:.1f}s -> arm {args.arm_hold:.1f}s -> "
        f"hover {args.hover:.1f}s @ z={args.altitude}m -> land {args.land_hold:.1f}s -> disarm")

    deadline = time.monotonic() + args.connect_timeout
    client = wait_for_client(server, stop, deadline)
    if client is None:
        if stop[0]:
            log("stop signal before client connected; exiting clean")
            rc = 0
        else:
            log(f"no client connected within {args.connect_timeout}s; exiting")
            rc = 1
        try: server.close()
        except Exception: pass
        _unlink(PID_FILE)
        return rc

    log("drone connected, entering sequence")

    send_buf = np.zeros(PACKET_FLOATS, dtype=np.float64)
    recv_buf = np.zeros(1, dtype=np.float64)
    recv_poll_to = Timeout(seconds=0, nanoseconds=100_000)  # 100 us

    t_preflight_end = args.preflight
    t_arm_end       = t_preflight_end + args.arm_hold
    t_hover_end     = t_arm_end + args.hover
    t_land_end      = t_hover_end + args.land_hold
    t_total         = t_land_end + 1.0

    t0 = time.monotonic()
    next_tick = t0
    last_status = 0.0
    last_phase = None
    rx_ok = 0
    rx_err = 0

    while not stop[0]:
        now = time.monotonic()
        elapsed = now - t0

        if args.idle_only:
            phase, arm, takeoff, dz, is_tracking = "IDLE-ONLY", 0, 0, 0.0, 0
        elif elapsed < t_preflight_end:
            phase, arm, takeoff, dz, is_tracking = "IDLE",   0, 0, 0.0, 0
        elif elapsed < t_arm_end:
            phase, arm, takeoff, dz, is_tracking = "ARM",    1, 0, 0.0, 1
        elif elapsed < t_hover_end:
            phase, arm, takeoff, dz, is_tracking = "HOVER",  1, 1, args.altitude, 1
        elif elapsed < t_land_end:
            phase, arm, takeoff, dz, is_tracking = "LAND",   1, 0, 0.0, 1
        elif elapsed < t_total:
            phase, arm, takeoff, dz, is_tracking = "DISARM", 0, 0, 0.0, 1
        else:
            log("sequence complete")
            break

        if phase != last_phase:
            log(f"-> {phase}")
            last_phase = phase

        build_packet(arm, takeoff, is_tracking, dz, time.time(), send_buf)
        try:
            client.send_double_array(send_buf, PACKET_FLOATS)
            client.flush()
        except Exception as e:
            log(f"send err (phase={phase}): {e}")

        # non-blocking receive
        try:
            rdy = client.poll(recv_poll_to, PollFlag.RECEIVE)
            if rdy & PollFlag.RECEIVE:
                client.receive_double_array(recv_buf, 1)
                rx_ok += 1
        except Exception:
            rx_err += 1

        if now - last_status >= 1.0:
            last_status = now
            log(f"phase={phase} t={elapsed:5.2f}s arm={arm} takeoff={takeoff} "
                f"dz={dz:.2f} isT={is_tracking} rx_ts={recv_buf[0]:.3f} rx_ok={rx_ok} rx_err={rx_err}")

        next_tick += TICK_DT
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()  # drifted — reset baseline

    log("exiting, sending disarm tail")
    build_packet(0, 0, 0, 0.0, time.time(), send_buf)
    for _ in range(50):
        try:
            client.send_double_array(send_buf, PACKET_FLOATS)
            client.flush()
        except Exception:
            pass
        time.sleep(TICK_DT)

    try: client.shutdown()
    except Exception: pass
    try: client.close()
    except Exception: pass
    try: server.close()
    except Exception: pass
    _unlink(PID_FILE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
