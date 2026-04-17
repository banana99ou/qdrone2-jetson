# QDrone 2 Mission Server — Design Notes

**Goal**: Replace Quanser's Windows-only Mission Server with a Python implementation that runs on the drone's Jetson, so lift-off requires no Simulink and no ground PC.

**Hardware state**: QDrone 2 (`qdrone-54334`, 192.168.0.169), Jetson L4T R32.7.3 aarch64, QUARC runtime 1.23-130, Python HIL SDK at `/usr/lib/python3/dist-packages/quanser/hardware/`.

---

## Phasing

| Phase | Scope | Deliverable |
|---|---|---|
| **1 — Minimal lift-off** | Hardcoded arm→takeoff→hover→land sequence on drone. No ground PC, no VRPN, no waypoints. | Drone lifts to fixed altitude, hovers, lands. Python mission server runs on Jetson. |
| **2 — Joystick-compatible** | Full port 18003 compatibility with Quanser's `joystick_commands.py` commander protocol. | Existing Quanser Python commander UX (arm/takeoff/land prompts) works unchanged. |
| **3 — Full waypoint + VRPN** | Trajectory streaming (port 18004) + external pose ingest for position hold. | Fly arbitrary paths from CRISP planner; supports VRPN/OptiTrack/custom pose source. |

---

## Current architecture (Quanser stock)

```
┌─────────── DRONE (Jetson) ──────────┐     ┌──────── GROUND PC (Windows) ─┐
│                                      │     │                              │
│  QD2_DroneStack_PID_2021a binary     │     │  Mission Server (Simulink)   │
│   ├─ Stream Server 18491 ─ telemetry │     │   ┌─ Stream Server 18001 ◀──┼─ DroneStack dials out
│   │   (QDrone2StreamStack.py reads)  │     │   │                          │
│   └─ Stream Client ──────────────────┼────▶│   │                          │
│        to -URI_Host = <ground>:18001 │     │   ├─ Stream Server 18003 ◀──┼─ JoystickCommands.py
│                                      │     │   └─ Stream Server 18004 ◀──┼─ WaypointTrajectory.py
│  HIL card: gyro/accel/mag in,        │     │                              │
│    motors out (ch 17000-17006)       │     │                              │
│                                      │     │                              │
│  (optional) Stream Client to         │     │                              │
│    VRPN pose at <ground>:18669       │     │                              │
└──────────────────────────────────────┘     └──────────────────────────────┘
```

Our target: replace the right-hand box with a Python process on the drone itself. Launch DroneStack with `-URI_Host tcpip://localhost:18001` so it dials into our local Python server.

---

## Pre-compiled binaries on the drone (`/var/spool/quarc/`)

| Binary | Size | Role | Use in phase 1? |
|---|---|---|---|
| `QD2_DroneStack_PID_2021a` | 285 KB | Onboard stabilizer (rate+attitude PID, motor mix) | **Yes** — this is the piece we keep |
| `QDrone2_PositionControl_2021b` | 285 KB | Position controller (bundled?) | Possibly in phase 3 |
| `QD2_IOCheck_2021b` / `QDrone2_IO_Check_*` | 238–242 KB | IO/motor-mapping verification | Pre-flight sanity check |
| `QD2_CameraCheck_2021b` / `QDrone2_CameraCheck_*` | 1.4 MB | Camera test | Not needed for flight |
| `QD2_cameras_2021a` | 1.4 MB | Camera-only model | Not needed |

`drone.py hw run <name>` already wraps `quarc_run` to launch any of these. **Per motor-safety rule: no running without explicit approval.**

---

## Key findings from static `.slx` decode

Source: `Quanser_Academic_Resources/5_research/autonomous_vehicles/qdrone2/python_to_simulink/hardware/Simulink/*.slx` (unzipped to `/Volumes/Sandisk/code/Q_Drone2/slx_extracted/`).

### DroneStack (`system_14085.xml`)

- **Stream Server** at `tcpip://localhost:18491?nagle='off'` — telemetry out (read by `pal.products.qdrone2.QDrone2StreamStack`). 47-element float64 packet already documented in that class: IMU raw ×2, state vectors ×3, motor cmd, battery, timestamp.
- **Stream Client** at `-URI_Host` (external, supplied by `quarc_run` launch args) — this is the mission-server connection. `PortCounts in="3" out="5"` → 3 signals going to server, 5 signals coming back.
- **HIL `other_channels`**: `[3000:3002 4000:4002 3003:3005 4003:4005 17000:17006 17008:17009 17007]`
  - 3000-3002: gyro (rad/s, 3 axes)
  - 4000-4002: accel (m/s², 3 axes)
  - 3003-3005: magnetometer (3 axes)
  - 4003-4005: secondary accel (second IMU)
  - 17000-17006: motor PWM outputs
  - 17007-17009: likely LED/aux outputs
- Default ground PC IP in configSet0: `192.168.2.56`, drone: `192.168.2.12`.

### MissionServerLegacy (`system_4279.xml`, `system_5687.xml`, `system_3974.xml`)

- **Stream Server** on `tcpip://localhost:18001` — accepts DroneStack's inbound connection.
- **Stream Server** on `tcpip://localhost:18003` — accepts JoystickCommands (5-element float64: `[arm, takeoff, estop, joystickIssue, mode]`).
- **Stream Server** on `tcpip://localhost:18004` — accepts WaypointTrajectory (8-element float64: `[x, y, z, roll, pitch, yaw, IsTracking, mode]`).
- Auxiliary sockets at 18800, 18877, 18987 (likely internal / legacy).

---

## What's still unknown

To write the phase-1 server we need the exact byte layouts of the 18001 channel. The `.slx` XML gives us the Stream Client block's port count (3 in / 5 out) but we need to trace which signals connect to those ports. Next static-analysis tasks:

1. **Parse `DroneStack/simulink/systems/system_14085.xml`** around SID 17376 (Stream Server at 18491) and SID near the Stream Client to `-URI_Host` — identify the signals wired to each input/output port of the Stream Client block.
2. **Parse `MissionServerLegacy/simulink/systems/system_4279.xml`** for the mirror side — what the Mission Server sends back to DroneStack on 18001, and what it consumes.
3. **Cross-reference with `commander.py` and `stream_stack_client_qd2.py`** (Quanser Python) for any hints on buffer sizes and dtypes.

`pal.utilities.stream.BasicStream` is available on the drone and handles the Quanser stream protocol (framing, buffering, connection retries). We can reuse it directly — no need to implement from scratch.

---

## Phase 1 plan (minimal lift-off)

### Prerequisites (non-code)
- Props OFF until static protocol decode + first bench test of server↔DroneStack handshake.
- Battery charged, drone clamped to bench, E-stop reachable.
- Quanser Python SDK accessible (`from quanser.common import Timeout` / `from pal.utilities.stream import BasicStream` — already works per `imu_service.py`).

### Code components
1. **`mission_server.py`** (new) — standalone Python process on the drone.
   - `BasicStream` listener on `tcpip://0.0.0.0:18001` (or `localhost` since same host).
   - State machine: `IDLE → ARMED → TAKING_OFF → HOVER → LANDING → IDLE`.
   - Hardcoded sequence (no keyboard input for phase 1): after handshake with DroneStack, wait 2 s, send ARM, wait 2 s, send TAKEOFF, hold for N seconds, send LAND.
   - Sends the 5-back signal to DroneStack matching the byte layout we extract in the "unknown" step.
2. **`drone.py mission` subcommand** — add a start/stop/status wrapper like `drone.py imu`. Launches `mission_server.py` under `sudo` if HIL access is needed (probably not — it's a TCP process).
3. **`drone.py fly` subcommand** — orchestrator that does:
   a. Verify no other mission server / DroneStack is running.
   b. Launch `mission_server.py` as background process.
   c. Launch `QD2_DroneStack_PID_2021a` via `quarc_run` with `-URI_Host tcpip://localhost:18001`.
   d. Tail logs from both, report connection handshake.
   e. On Ctrl-C: send LAND, wait for descent, then shut both down.

**Bold rule: `drone.py fly` never executes without explicit user approval per this session's motor-safety rule.**

### Bring-up order (props OFF)
1. Run `mission_server.py` standalone — verify it binds 18001 and logs connections.
2. Run `QD2_DroneStack_PID_2021a` separately (requires approval — motor risk even without commander if the model arms on startup).
3. Confirm both processes stay alive, stream traffic, and the motors do NOT spin without an ARM command from the server.
4. Only with props-off sanity confirmed, progress to ARM → TAKEOFF with props on (with approval for every step).

---

## Open questions for the user

- **OK to run DroneStack alone with props off just to observe handshake?** (Low risk — no commander means no arm.) Decision gate before going further.
- **Phase 2/3 features to defer?** Confirming waypoints and VRPN are strictly phase 3 so I don't over-engineer phase 1 APIs.
- **Log location**: reuse `~/ros2/qdrone/logs/mission.log` via the existing `drone.py` bg-service pattern? Seems natural.

---

## File locations

- This doc: `/Volumes/Sandisk/code/Q_Drone2/mission_server_design.md` (local Mac, to be copied to drone repo `~/ros2/qdrone/docs/` when approved)
- Unzipped .slx source: `/Volumes/Sandisk/code/Q_Drone2/slx_extracted/`
- Precompiled binaries: `/var/spool/quarc/` on drone
- Existing Python services: `~/ros2/qdrone/{drone.py,imu_service.py,cam_stream.py}` on drone
