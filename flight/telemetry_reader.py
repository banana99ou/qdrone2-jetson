#!/usr/bin/env python3
"""Live telemetry reader for DroneStack's Stream Server (port 18491).

DroneStack exposes a 47-double telemetry packet per tick. This tool connects
as a client, receives the stream, and prints motor commands + battery + a
slice of state at ~1 Hz. Use it to diagnose what the stabilizer is actually
outputting while mission_server drives the command side.

Packet layout (from pal.products.qdrone2.QDrone2StreamStack, 47 × float64):
    [0..5]   imu_0_raw       gyro(3, rad/s) + accel(3, m/s^2) for IMU 0
    [6..11]  imu_1_raw       IMU 1 (same layout)
    [12..20] state_vec_0     9-element state (IMU 0 path)
    [21..29] state_vec_1     9-element state (IMU 1 path)
    [30..38] state_vec       combined 9-element state
    [39]     motorCurrent
    [40..43] motorCmd        4-motor command (unit TBD — likely normalized PWM)
    [44]     battVoltage
    [45]     lowBattery
    [46]     timestampREC

Run in parallel with mission_server + DroneStack. Read-only. No motor risk.
"""
import argparse
import sys
import time

import numpy as np

try:
    from quanser.communications import Stream, PollFlag
    from quanser.common import Timeout
except ImportError as e:
    print(f"[tel] missing Quanser python API: {e}", flush=True)
    sys.exit(2)


PACKET_SIZE = 47
DEFAULT_URI = "tcpip://localhost:18491"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=2.0,
                    help="status print rate [Hz]")
    args = ap.parse_args()

    send_buf = np.zeros(1, dtype=np.float64)
    recv_buf = np.zeros(PACKET_SIZE, dtype=np.float64)

    client = Stream()
    try:
        # blocking connect so we retry until DroneStack is up
        client.connect(args.uri, False, 2048, 2048)
    except Exception as e:
        print(f"[tel] connect failed: {e}", flush=True)
        return 1

    print(f"[tel] connected to {args.uri}", flush=True)
    print(f"[tel] col: t  mcmd0..3  imu_gyro  imu_accel  batt  lowBatt  droneTS", flush=True)

    t0 = time.monotonic()
    last_print = 0.0
    poll_to = Timeout(seconds=0, nanoseconds=1_000_000)
    ok = 0
    err = 0

    while time.monotonic() - t0 < args.duration:
        try:
            send_buf[0] = time.time()
            client.send_double_array(send_buf, 1)
            client.flush()
            # non-blocking receive
            rdy = client.poll(poll_to, PollFlag.RECEIVE)
            if rdy & PollFlag.RECEIVE:
                client.receive_double_array(recv_buf, PACKET_SIZE)
                ok += 1
        except Exception as e:
            err += 1
            if err % 50 == 1:
                print(f"[tel] err #{err}: {e}", flush=True)

        now = time.monotonic()
        if now - last_print >= 1.0 / args.rate:
            last_print = now
            mcmd = recv_buf[40:44]
            gyro = recv_buf[0:3]
            accel = recv_buf[3:6]
            batt = recv_buf[44]
            low = recv_buf[45]
            ts = recv_buf[46]
            print(f"t={now-t0:5.2f}s  "
                  f"mcmd=[{mcmd[0]:+.3f}, {mcmd[1]:+.3f}, {mcmd[2]:+.3f}, {mcmd[3]:+.3f}]  "
                  f"gyro=[{gyro[0]:+.2f},{gyro[1]:+.2f},{gyro[2]:+.2f}]  "
                  f"accel=[{accel[0]:+.2f},{accel[1]:+.2f},{accel[2]:+.2f}]  "
                  f"batt={batt:.2f}V  low={int(low)}  ts={ts:.3f}  "
                  f"rx_ok={ok} rx_err={err}",
                  flush=True)
        time.sleep(0.004)  # ~250 Hz

    try: client.shutdown()
    except Exception: pass
    try: client.close()
    except Exception: pass
    print(f"[tel] done. rx_ok={ok} rx_err={err}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
