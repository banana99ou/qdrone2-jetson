# Development Journal

Newest entries at the top. Each entry = one working session. Keep it honest — "did not work, here's what we saw" beats optimistic summaries.

---

## 2026-04-17 — Phase-1 bench-test attempt (NOT complete)

_Half-day session. QDrone 2 at 192.168.0.169. Props off throughout._

### 1. Existing drone control infrastructure (what was already there)

**Hardware/OS**
- Jetson-based QDrone 2, L4T R32.7.3 aarch64, hostname `qdrone-54334`
- Intel RealSense D435 camera, VL53L1X ToF sensor (I2C), QDrone IMU set (2× IMU + magnetometer)

**Quanser stack installed (dpkg packages `quanser-*`, `hil-*`, `python3-quanser-*`)**
- `quarc_target_manager` daemon on port 17000 — receives model load/run commands (systemd `Type=forking`)
- `quanser_license_manager` daemon on port 16999
- `quanser.hardware`, `quanser.communications`, `quanser.common` Python APIs
- **No `pal.*` package** — that ships with Quanser's Research Resources zip, not installed here
- HIL SDK libraries `/usr/lib/aarch64-linux-gnu/quanser/hardware/*.so` including `qdrone2.so`

**Precompiled QUARC model binaries** in `/var/spool/quarc/` (aarch64 ELFs, mode 700 root:root)
- `QD2_DroneStack_PID_2021a` — stabilizer (285 KB)
- `QDrone2_PositionControl_2021b` — position controller (285 KB)
- `QD2_IOCheck_2021b`, `QDrone2_IO_Check_2021b`, `QDrone2_IO_Check_Jan2023` — motor mapping tests
- `QD2_CameraCheck_2021b`, `QDrone2_CameraCheck_2021b`, `QDrone2_CameraCheck_Jan2023`, `QD2_cameras_2021a` — camera tests

**Pre-existing repo code** (`~/ros2/qdrone/`, from colleague/previous work)
- `imu_service.py` — HIL IMU reader → complementary filter → UDP publisher on 127.0.0.1:8090
- `cam_stream.py` — RealSense MJPEG + point-cloud HTTP streamer
- `drone.py` — unified CLI (subcommands: info, cam, hw, imu, sim)
- `planner/` submodule — CRISP SFC/Bézier planner in C++

---

## 2. What we built this session

- **GitHub repo** `banana99ou/qdrone2-jetson`, synced Mac ↔ drone
- **`flight/__init__.py`** + **`flight/mission_server.py`** — Python replacement for Quanser's Simulink Mission Server. TCP `0.0.0.0:18001`, QUARC stream protocol via `quanser.communications.Stream`, 16 × float64 per tick at ~250 Hz. Hardcoded state machine (IDLE → ARM → HOVER → LAND → DISARM). CLI flags for altitude, hover duration, idle-only mode, connect timeout.
- **`flight/telemetry_reader.py`** — read-only client targeting DroneStack's telemetry Stream Server at port 18491 (doesn't currently work — see observations).
- **`drone.py mission` subcommand** — start/stop/status/logs wrapper following existing bg-service pattern.
- **`docs/mission_server.md`** — design doc + protocol decode from the .slx XML
- **`docs/knowhow.md`** — operational knowhow, failure modes, recovery cheatsheet
- **`/var/spool/quarc/*.cp`** backup copies on the drone (every flight binary has a `.cp` next to it, after we destroyed one binary and had to restore from a donor drone)

---

## 3. What we tried (chronological)

| # | Action | Outcome |
|---|---|---|
| 1 | `quarc_run -r` with symlink `.rt-linux_qdrone2` → real binary in same dir | **Destroyed the binary** — 0 bytes. `-r` opened source for write via symlink, truncated before reading |
| 2 | Restored binary from another QDrone 2 (you scp'd donor's spool locally, I pushed back) | Binary restored, 285232 B, working |
| 3 | `quarc_run -l` with binary in /tmp | Hung silently; no model process; no logs |
| 4 | `quarc_run -r` from /tmp source | "Error 0. No corresponding error message." |
| 5 | `pkill -f 'quarc_run'` to clean stale procs | **Killed my own shell** (cmdline contained `quarc_run` as substring); brief motor sound during the chain |
| 6 | `quarc_run -D -r` backgrounded via setsid | Same silent hang, no output |
| 7 | `-c` console in parallel with `-D -r` | Same hang |
| 8 | `nc hello` to port 17000 to probe daemon | **Daemon closed its listener permanently** — had to restart |
| 9 | `systemctl restart quarc_target_manager` | Daemon fresh; `quarc_run -T` returned `linux_qdrone2`; client/server protocol works again |
| 10 | `quarc_run -D -r` fresh, no `-uri` arg | Model I/O threads run, mission server sees sequence complete at 250 Hz. **Motors never spin.** |
| 11 | Added `is_tracking=1` in ARM/HOVER/LAND/DISARM phases | Beep pattern changed slightly, but still no motor spin |
| 12 | Added `-uri tcpip://localhost:17001?retries=10` (Quanser's own arg pattern) | No change |
| 13 | Added `-S` MEX arg (skip ExtMode `-w` wait) | No change |

---

## 4. What we observed (facts)

### Protocol — confirmed working end-to-end
- Mission server binds TCP 18001, accepts connection from DroneStack
- DroneStack sends 1 double per tick (timestamp echo); server sends 16 doubles per tick
- `rx_ok` climbs at ~250 Hz throughout all 17 seconds of the state machine. Zero `rx_err`.
- Server-side sequence state machine fires on schedule (wall clock)

### Binary behavior — partial / gated somewhere
- `quarc_run -D -r` returns cleanly (exit 0) after our 30 s `timeout` wrapper
- DroneStack's Stream Client I/O thread is alive (rx_ok proves it receives our packets)
- **`rx_ts` (drone's echoed timestamp) updates during IDLE/ARM, then freezes exactly at the HOVER transition** and stays frozen through LAND/DISARM
- **Port 18491 (DroneStack's telemetry Stream Server per the .slx source) is never bound at runtime** — `netstat` confirms; our telemetry reader fails to connect
- **Motors get no PWM signal** — ESCs produce continuous idle beep (user's hands-on experience: this pattern = "no signal at all")
- `strings` on the binary shows only 4 URIs: `i2c-cpu://localhost:0` (ToF sensor), `stdio://localhost:1` (console), `tcpip://192.168.2.25:17001?retries=10` (default ExtMode URI), `shmem://mymodel` (help text). **The `tcpip://localhost:18491` from the .slx XML is NOT a literal string in the binary.**

### QUARC daemon — fragile
- Malformed client input (like `nc`'s garbage) causes it to close its listen socket permanently
- Recovery: `sudo systemctl restart quarc_target_manager`
- `pkill -f` with a pattern matching your own shell cmdline causes collateral damage (kills the shell)

### ESC behavior
- ESC switch does NOT control the beep (confirmed by user)
- Beep = ESCs powered with no PWM signal; stops only on physical power-down or valid PWM
- We have never driven valid PWM from the binary

---

## 5. Bottom-line interpretation

Our mission server works perfectly at the protocol level. The stabilizer binary's I/O thread happily receives our commands. **But the binary's main step loop either doesn't run or doesn't reach the motor-PWM output stage.**

Evidence pointing there:
1. `rx_ts` freezes at HOVER (drone stops echoing current timestamps — something upstream of the send-path stops stepping)
2. Port 18491 never binds (telemetry Stream Server never starts)
3. Zero PWM reaches the motors (ESCs never get a signal)

Phase 1 (motor actually spinning) is **NOT complete**. What we have is "protocol flow works" — necessary but not sufficient.

---

## 6. Open hypotheses — ranked by my current confidence

1. **Main step gated on something we're not satisfying.** Not ExtMode `-w` (we tried `-S`), not `is_tracking` (we tried that). Could be: trigger byte order wrong, a comm-loss latch from one of our earlier botched attempts, a battery-voltage sanity gate, or another condition I haven't identified yet.
2. **The binary needs something in the Simulink runtime environment** that `quarc_run` normally provides when paired with an ExtMode console (`-c`). We tried running `-c` in parallel once but the daemon was damaged at that time; haven't retried cleanly since daemon restart.
3. **The `.rt-linux_qdrone2` binary from the donor drone doesn't match this drone's specific calibration/config.** Less likely — QDrone 2s are same hardware, same binary version — but not zero.
4. **Model arming requires a transient on specific triggers (edge-triggered, not level-triggered).** Level-hold at arm=1 might not be recognized as a distinct "arm now" event.

## 7. Next moves I'd suggest (no execution until you say so)

- **Watch `journalctl -u quarc_target_manager -f` DURING a `quarc_run -D -r`.** I never did this in sync. The daemon logs every accept/download/load — if the model loads but something about it is rejected, the log will say so.
- **Run Quanser's ground-truth `q_setup.py` or equivalent** on a Windows machine (if you have one) to confirm the binary + your drone lift off with Quanser's stock setup. If yes, the gap is in our setup; if no, the drone is in an unusual state.
- **Try `quarc_run -c` console in parallel on a fresh daemon.** Properly this time. The theory that main step waits for ExtMode is still alive — `-S` might not be respected by this particular build.
- **Check trigger byte order empirically** — send a single 1 in each trigger position (arm-only, takeoff-only, estop-only, ji-only) and see which ones cause any motor response. Cheap experiment.
- **Use `QDrone2_PositionControl_2021b` as an alternative stabilizer** (same size, presumably related, also on disk, intact) and see if it has different behavior.

---

_End of journal. Not a claim of progress — a map of where we are._
