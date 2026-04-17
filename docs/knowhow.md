# QDrone 2 Phase-1 Knowhow

Central reference for operational quirks, mistakes made, and known-working recipes. Consult this before any QUARC/HIL-related work on the drone.

---

## TL;DR — the proven working recipe

With **props off** and mission server in full-sequence mode:

```bash
# On drone:
cd ~/ros2/qdrone
./drone.py mission stop
./drone.py mission start --connect-timeout 120

# Copy binary out of spool each time (do NOT symlink into spool):
sudo cp /var/spool/quarc/QD2_DroneStack_PID_2021a \
        /tmp/QD2_DroneStack_PID_2021a.rt-linux_qdrone2

# Launch DroneStack (foreground so you see output):
sudo /usr/bin/quarc_run -D -r -t tcpip://localhost:17000 \
    /tmp/QD2_DroneStack_PID_2021a.rt-linux_qdrone2 \
    -URI_Host tcpip://localhost:18001
```

Mission server runs the full sequence: IDLE 3 s → ARM 3 s → HOVER 5 s @ z=0.5 m → LAND 5 s → DISARM. Total ~17 s. Motors command as expected; with props off they'll spin freely.

Verify success in mission log:
```bash
grep -E 'drone connected|-> |sequence complete' ~/ros2/qdrone/logs/mission.log | tail -20
```
Expected: `drone connected` → `-> IDLE` → `-> ARM` → `-> HOVER` → `-> LAND` → `-> DISARM` → `sequence complete`. `rx_ok` counter should climb by ~250 per second throughout.

---

## QUARC target_manager — the fragile listener

**`quarc_target_manager` closes its listen socket if any client sends malformed protocol.** The daemon process stays alive, but no new clients can connect. Symptoms: `quarc_run -T` hangs with no output, nothing in the daemon's journal about your attempts, `nc -zv localhost 17000` says "Connection refused".

### How to tell it's in the broken state

```bash
sudo netstat -tlnp | grep 17000
```
- If you see `LISTEN ... quarc_target_m` → daemon listening, OK.
- If nothing → daemon closed its listener, you can't talk to it.

Also the daemon's own journal will show the proof:
```
sudo journalctl -u quarc_target_manager -n 20 --no-pager
```
Look for: `failed to accept a client connection ... The DPC connection has been closed at the peer. Closing service on tcpip://localhost:17000`.

### Recovery

```bash
sudo systemctl restart quarc_target_manager
```
That's it. Daemon recovers a listening socket.

### What NOT to do

- **Never `nc 127.0.0.1 17000`** or any other protocol-raw probe. The first byte of garbage closes the listener.
- **Never chain-kill `quarc_run` processes** while the daemon has active sessions — half-torn-down sessions may also trip the listener-close.
- If you have multiple `quarc_run` processes hanging and you can't tell which one poisoned the daemon, just restart the daemon after killing them.

### Sane ways to probe

`quarc_run -T ... <some.rt-linux_qdrone2>` will print the target type (`linux_qdrone2`) and return cleanly with a non-zero exit (because the model file may be a dummy). Safe health-check.

---

## `quarc_run` invocation — what works

### The right verb

- **`-D -r`** = download + run. This is Quanser's own pattern in `q_setup.py`. Proven working.
- `-l` alone = load already-downloaded model. **Hangs** on this drone image — the model never starts, no process, no port listeners. Don't use.
- `-r` alone = same as `-D -r` conceptually, but historically hit the self-overwrite bug (see below).
- `-c` = attach console to running model. Streams the model's stdout. Useful for debugging. Will block until Ctrl-C.
- `-q` = stop a running model. Requires model name matching what's running.
- `-T` = print target type. Safe, returns quickly. Good liveness check.

### Required args

- `-t tcpip://<host>:17000` — target daemon URI.
- `<path-to-binary>.rt-linux_qdrone2` — the model binary. **Must have the `.rt-linux_qdrone2` extension**, else `quarc_run -r` errors "A file extension must be given on the model name". The bare files in `/var/spool/quarc/` have no extension — you must copy (not symlink!) them to a writable path with the extension.
- `-URI_Host tcpip://<host>:<port>` — passed through to the model as a MEX argument; this is where DroneStack dials its Stream Client. Point it at the mission server (e.g. `tcpip://localhost:18001`).
- `-uri tcpip://...:17001?retries=10` — optional; per-model ExtMode URI. Not required for headless operation.

### Foreground vs background

- **Run it foreground**. The process blocks until the model stops — that's expected, it's supervising the model. If you need to do other things, open another terminal (or tmux pane).
- Backgrounding it (`sudo setsid ... &`, `nohup ... &`) was tried; caused more confusion than it solved. Just use a second SSH session.

---

## Binary handling — the footguns

### Mistake 1 — `-r` with a symlink pointing into the spool **destroys the source**

I did this once and cost us the `QD2_DroneStack_PID_2021a` binary. Recovered by copying from another QDrone 2.

- `/var/spool/quarc/QD2_DroneStack_PID_2021a` — the real binary, root:root mode 700.
- I symlinked `QD2_DroneStack_PID_2021a.rt-linux_qdrone2 → QD2_DroneStack_PID_2021a` inside the same directory to satisfy `quarc_run`'s "must have extension" check.
- Ran `quarc_run -r -t tcpip://localhost:17000 <the symlink>`.
- `quarc_run -r` opened the source path (via symlink) **for write** during its "download" step, truncating the binary to 0 bytes before the read could happen.
- Symptoms: "Error 0 occurred. No corresponding error message."

**Rule**: Never symlink a `.rt-*` extension pointing back to the original in the same directory. Always **copy** to `/tmp/` (or some path outside the spool) instead:
```bash
sudo cp /var/spool/quarc/QD2_DroneStack_PID_2021a \
        /tmp/QD2_DroneStack_PID_2021a.rt-linux_qdrone2
```

### Mistake 2 — `/tmp` can be cleaned at unexpected times

The binary in `/tmp` disappeared between sessions (probably tmpfs cleanup or a side-effect of a failed `-r`). Re-copying is cheap; always verify with `ls -la /tmp/QD2_*` before launching.

### The backup strategy

`/var/spool/quarc/<name>.cp` duplicates were made for every flight binary. If the canonical copy ever gets destroyed again:
```bash
sudo cp /var/spool/quarc/QD2_DroneStack_PID_2021a.cp \
        /var/spool/quarc/QD2_DroneStack_PID_2021a
sudo chown root:root /var/spool/quarc/QD2_DroneStack_PID_2021a
sudo chmod 700 /var/spool/quarc/QD2_DroneStack_PID_2021a
```

---

## `pkill -f` self-match — destructive footgun

Ran `sudo pkill -9 -f 'quarc_run'` inside an ssh command. The pattern matched the remote bash shell's own cmdline (because the ssh command string contained `quarc_run`). SIGKILL'd the shell. Weird side-effects, brief motor sound.

**Rule**: never `pkill -f` with a pattern that appears in your own command. Use:
- `pkill -x quarc_run` (matches the short `comm` name — `bash` won't match)
- `pkill -f '^/usr/bin/quarc_run'` (anchored regex)
- `kill <specific-PID>` after `ps | grep` filter

---

## Mission server (`flight/mission_server.py`)

### Protocol (decoded, verified)

Port 18001 TCP, little-endian float64, no framing. Server binds; DroneStack connects as client.

| Direction | Size | Layout |
|---|---|---|
| Server → Drone | 128 B (16 × f64) | `[arm, takeoff, estop, joystick_issue, is_tracking, mx, my, mz, mr, mp, my, dx, dy, dz, dyaw, timestamp]` |
| Drone → Server | 8 B (1 × f64) | `[timestamp echo]` |

Rate: 250 Hz (model has a Rate Transition labeled "slow from 500hz to 250Hz" on the Stream path). Target 4 ms tick in the server.

### Library

`quanser.communications.Stream` (installed via `python3-quanser-communications` package). **`pal.utilities.stream.BasicStream` is NOT installed** on this drone image — it ships separately with Quanser's Research Resources zip. Use the low-level `Stream` class directly; `accept`, `send_double_array`, `receive_double_array`, `poll(..., PollFlag.RECEIVE|ACCEPT)` are all there.

### CLI flags

- `--idle-only` — stay in IDLE phase forever (arm=0). Useful for handshake-only tests.
- `--altitude <m>` — hover altitude (default 0.5).
- `--hover <s>` — hover duration (default 5.0).
- `--preflight`, `--arm-hold`, `--land-hold` — per-phase durations.
- `--connect-timeout <s>` — how long to wait for DroneStack to connect (default 30 s; set to 120+ in practice because bench-test dance takes time).

### `drone.py mission` subcommand wraps all of the above.

---

## Known model quirks (to investigate in phase 2)

### `rx_ts` freezes after HOVER transition

Observed: drone's timestamp loopback stops updating once the HOVER phase starts. `rx_ok` keeps climbing (TCP stays healthy), but `rx_ts` stays pinned at whatever value it was at the HOVER transition.

Likely cause: the Stream Server/Client inside DroneStack uses "Send most recent data" / "Receive most recent data" options, which means send packets may coalesce. Or the rate-transition + some state-dependent enable triggers off.

Non-blocking for phase 1 (command flow works regardless). Worth understanding before phase 3 if the timestamp is ever used for tracking control latency.

### ESC beeping vs motor spin — they sound similar

With ESCs powered (armed switch on) but no active PWM command, ESCs beep periodically (their "I'm alive, no signal" indicator). That's NOT motor actuation. Actual motor spin sounds different — continuous whine, not periodic beep.

If you hear anything during a test and aren't sure: ps + netstat will tell you if a model is actively sending commands. No command source = no motor spin, only beep.

---

## Safety protocol for bench testing

1. **Props off every time.** Don't even consider "just this once with props on".
2. **ESC switch: armed position is fine** with props off (lets the ESCs actually respond so we can verify command flow).
3. **E-stop within reach.** Any unexpected motor spin: kill power first, ask questions later.
4. **One change per test.** Don't bundle a binary update, a protocol change, and a sequence change into the same run.
5. **Keep the daemon journal open in another pane** during any `quarc_run` attempt. You'll know immediately if the listener got closed.
6. **After any `quarc_run` hang/crash: restart the daemon before retrying.** Don't trust a "maybe it recovered" state.

---

## Recovery cheatsheet

| Symptom | Likely cause | Fix |
|---|---|---|
| `quarc_run -T` hangs with no output | Daemon listener closed | `sudo systemctl restart quarc_target_manager` |
| `quarc_run -r` prints "Error 0, no message" | Source path resolves to a spool file via symlink | Copy (don't symlink) to `/tmp` with extension |
| `quarc_run: command not found` | Binary missing from `/tmp` (often cleaned) | Re-copy from `/var/spool/quarc/` |
| `quarc_run` says "must give extension" | Bare binary name without `.rt-linux_qdrone2` | Add the extension (via copy, not symlink!) |
| Motor spins when it shouldn't | Some stale quarc_run still holding a session | `sudo pkill -x quarc_run` then restart daemon |
| ESC beeping but motor not spinning | ESCs armed, no PWM signal | Normal — no action needed |
| Mission server log shows `rx_ok=0 rx_err=high` | Stream Server protocol mismatch | Check byte order, packet size, dtype |
| Mission server log shows `no client connected within Xs` | DroneStack never dialed in | Verify `-URI_Host` arg matches the mission server port; verify daemon is listening |

---

## Files and paths (drone-side)

```
/var/spool/quarc/QD2_DroneStack_PID_2021a           stabilizer binary (root:root 700)
/var/spool/quarc/QD2_DroneStack_PID_2021a.cp        backup copy (belt-and-suspenders)
/var/spool/quarc/QDrone2_PositionControl_2021b      position controller (phase 3 candidate)
/usr/bin/quarc_run                                  client
/usr/sbin/quarc_target_manager                      daemon (port 17000)
/usr/sbin/quanser_license_manager                   license daemon (port 16999)
/lib/systemd/system/quarc_target_manager.service    daemon systemd unit
/usr/lib/python3/dist-packages/quanser/              Python HIL/comms APIs
/tmp/QD2_DroneStack_PID_2021a.rt-linux_qdrone2      scratch copy for quarc_run (re-copy each session)
~/ros2/qdrone/flight/mission_server.py              our Python mission server
~/ros2/qdrone/drone.py                              management CLI (cam/imu/sim/mission/hw)
~/ros2/qdrone/logs/mission.log                      mission server output
```

---

_Last updated: 2026-04-17 after phase-1 lift-off sequence validated end-to-end (commit after this doc)._
