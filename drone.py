#!/usr/bin/env python3
"""QDrone 2 management CLI — single entry point for all drone operations.

Usage:
    ./drone.py <command> [options]
    ./drone.py --help
    ./drone.py <command> --help

Layout of this workspace (everything lives under this directory):
    drone.py          - this file (CLI entry point)
    cam_stream.py     - RealSense MJPEG + point-cloud streamer (launched by 'cam')
    logs/             - per-service log files (auto-created)
    .pids/            - pid tracking for background services (auto-created)

Add new subsystems as new subcommands in the COMMANDS section below.
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# ----------------------------------------------------------------------------
# paths & constants
# ----------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent
PID_DIR = ROOT / ".pids"
LOG_DIR = ROOT / "logs"
PID_DIR.mkdir(exist_ok=True)
LOG_DIR.mkdir(exist_ok=True)

STREAMER_SCRIPT = ROOT / "cam_stream.py"
STREAMER_PORT = 8080
IMU_SCRIPT = ROOT / "imu_service.py"
MISSION_SCRIPT = ROOT / "flight" / "mission_server.py"
MISSION_PORT = 18001
SIM_BIN = ROOT / "planner" / "Simulation" / "C++" / "drone_sim"
SIM_CONFIG = ROOT / "planner" / "Simulation" / "C++" / "config.jsonc"
SIM_DISPLAY = ":0"
SIM_XAUTH = "/run/user/1000/gdm/Xauthority"

QUARC_SPOOL = Path("/var/spool/quarc")
QUARC_RUN = "/usr/bin/quarc_run"
QUARC_URI = "tcpip://localhost:17000"

# ----------------------------------------------------------------------------
# background-service helpers (start/stop/status with pid files)
# ----------------------------------------------------------------------------
def _pid_file(name): return PID_DIR / f"{name}.pid"
def _log_file(name): return LOG_DIR / f"{name}.log"


def _unlink(path):
    try:
        path.unlink()
    except FileNotFoundError:
        pass


def _running_pid(name):
    f = _pid_file(name)
    if not f.exists():
        return None
    try:
        pid = int(f.read_text().strip())
    except ValueError:
        _unlink(f)
        return None
    try:
        os.kill(pid, 0)
        return pid
    except ProcessLookupError:
        _unlink(f)
        return None
    except PermissionError:
        # Process exists but we lack signal rights (e.g. owned by root).
        # Keep the pid file and report the pid — stop goes through sudo.
        return pid


def _start_bg(name, cmd):
    existing = _running_pid(name)
    if existing:
        print(f"[{name}] already running (pid {existing})")
        return existing
    log = open(_log_file(name), "ab")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, cwd=str(ROOT))
    _pid_file(name).write_text(str(proc.pid))
    print(f"[{name}] started pid {proc.pid}")
    print(f"       log: {_log_file(name)}")
    return proc.pid


def _stop_bg(name):
    pid = _running_pid(name)
    if pid is None:
        print(f"[{name}] not running")
        return
    try:
        os.killpg(os.getpgid(pid), signal.SIGTERM)
    except ProcessLookupError:
        os.kill(pid, signal.SIGTERM)
    for _ in range(30):
        time.sleep(0.1)
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
    else:
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    _unlink(_pid_file(name))
    print(f"[{name}] stopped")


def _tail_log(name, n=50, follow=False):
    f = _log_file(name)
    if not f.exists():
        print(f"[{name}] no log yet at {f}")
        return
    args = ["tail", "-n", str(n)]
    if follow:
        args.append("-f")
    args.append(str(f))
    subprocess.run(args)


# ============================================================================
# COMMANDS
# ============================================================================

# ---- info -----------------------------------------------------------------
def cmd_info(args):
    print("=== QDrone 2 system ===")
    u = os.uname()
    print(f"host      : {u.nodename}")
    print(f"kernel    : {u.release}")
    print(f"arch      : {u.machine}")

    r = subprocess.run(["ip", "-4", "-o", "addr", "show"],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       universal_newlines=True)
    ips = []
    for line in r.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet":
            ip = parts[3].split("/")[0]
            if ip != "127.0.0.1":
                ips.append(f"{parts[1]}={ip}")
    print(f"ips       : {', '.join(ips) or 'none'}")

    try:
        r = subprocess.run(["dpkg-query", "-W", "-f=${Version}", "quarc-runtime"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, check=True)
        print(f"QUARC     : {r.stdout or 'installed (version unknown)'}")
    except subprocess.CalledProcessError:
        print("QUARC     : not installed")

    try:
        r = subprocess.run(["rs-enumerate-devices", "-s"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=5)
        for line in r.stdout.splitlines():
            if "Intel RealSense" in line:
                print(f"camera    : {line.strip()}")
                break
        else:
            print("camera    : not detected")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("camera    : (rs-enumerate-devices unavailable)")

    print()
    print("=== managed services ===")
    any_service = False
    for pf in sorted(PID_DIR.glob("*.pid")):
        any_service = True
        name = pf.stem
        pid = _running_pid(name)
        status = f"running pid {pid}" if pid else "stale"
        print(f"  {name:20s} {status}")
    if not any_service:
        print("  (none)")


# ---- cam (streamer) --------------------------------------------------------
def cmd_cam(args):
    if args.action == "start":
        pid = _start_bg("cam", [sys.executable, str(STREAMER_SCRIPT)])
        if pid:
            time.sleep(2)
            print()
            print(f"  open in browser (via SSH port-forward of :{STREAMER_PORT}):")
            print(f"    http://localhost:{STREAMER_PORT}/        2D color + depth")
            print(f"    http://localhost:{STREAMER_PORT}/pc      3D point cloud")
            print(f"    http://localhost:{STREAMER_PORT}/points  binary xyz+rgb")
    elif args.action == "stop":
        _stop_bg("cam")
    elif args.action == "status":
        pid = _running_pid("cam")
        print(f"cam : {'running pid '+str(pid) if pid else 'stopped'}")
    elif args.action == "logs":
        _tail_log("cam", n=args.n, follow=args.follow)


# ---- hw (QUARC binaries) ---------------------------------------------------
def cmd_hw(args):
    if args.action == "list":
        if not QUARC_SPOOL.exists():
            print(f"no quarc spool at {QUARC_SPOOL}")
            return
        print(f"=== pre-compiled QUARC models at {QUARC_SPOOL} ===")
        for p in sorted(QUARC_SPOOL.iterdir()):
            if p.is_file():
                print(f"  {p.name}   ({p.stat().st_size/1024:.0f} KB)")
        print()
        print("run with:  ./drone.py hw run <name>")
        return

    if not args.model:
        print("error: <model> required. list with: ./drone.py hw list")
        return

    target = QUARC_SPOOL / args.model
    if not target.exists():
        print(f"no such model: {args.model}")
        return

    if args.action == "run":
        cmd = [QUARC_RUN, "-r", "-t", QUARC_URI, str(target)]
    elif args.action == "stop":
        cmd = [QUARC_RUN, "-q", "-t", QUARC_URI, str(target)]
    elif args.action == "kill":
        cmd = [QUARC_RUN, "-k", "-t", QUARC_URI, str(target)]
    else:
        return
    print(f">> {' '.join(cmd)}")
    subprocess.run(cmd)


# ---- imu (Quanser HIL → UDP pose publisher) -------------------------------
#
# Runs imu_service.py as root (HIL hardware access requires root; error -11
# = QERR_NO_PERMISSION for a normal user). The service reads gyro + accel
# from qdrone2 HIL channels 3000-3002 and 4000-4002, runs a complementary
# filter for attitude, and UDP-publishes pose to 127.0.0.1:8090 where
# cam_stream.py picks it up and exposes /pose + updates the 3D viewer.
#
# The pid file is written by imu_service.py itself as root, so '_stop_bg'
# can't kill it (user can't signal a root process). Use 'sudo kill' via
# cmd_imu('stop') instead.
def _imu_pid():
    return _running_pid("imu")


def cmd_imu(args):
    if args.action == "start":
        if _imu_pid():
            print(f"[imu] already running (pid {_imu_pid()})")
            return
        log = open(_log_file("imu"), "ab")
        print("[imu] launching imu_service.py as root "
              "(enter sudo password if prompted)")
        # keep stdin attached for sudo password prompt; detach process group
        # so it survives the parent CLI exit.
        proc = subprocess.Popen(
            ["sudo", sys.executable, str(IMU_SCRIPT)],
            stdout=log, stderr=subprocess.STDOUT,
            stdin=sys.stdin, cwd=str(ROOT), start_new_session=True,
        )
        # wait briefly for the service to write its own pid file or fail
        for _ in range(50):
            time.sleep(0.1)
            if _imu_pid():
                print(f"[imu] started pid {_imu_pid()}")
                print(f"       log: {_log_file('imu')}")
                return
            if proc.poll() is not None:
                print(f"[imu] service exited early (rc={proc.returncode}); "
                      f"see {_log_file('imu')}")
                return
        print("[imu] still starting; check: ./drone.py imu status")

    elif args.action == "stop":
        pid = _imu_pid()
        if pid is None:
            print("[imu] not running")
            return
        print(f"[imu] stopping pid {pid} (sudo required)")
        subprocess.run(["sudo", "kill", str(pid)])
        for _ in range(30):
            time.sleep(0.1)
            if _imu_pid() is None:
                print("[imu] stopped")
                return
        print("[imu] still running after SIGTERM; escalating to SIGKILL")
        subprocess.run(["sudo", "kill", "-9", str(pid)])

    elif args.action == "status":
        pid = _imu_pid()
        print(f"imu : {'running pid '+str(pid) if pid else 'stopped'}")

    elif args.action == "logs":
        _tail_log("imu", n=args.n, follow=args.follow)


# ---- sim (drone_sim GLFW app, renders on Jetson's :0 display) --------------
def cmd_sim(args):
    if args.action == "start":
        if not SIM_BIN.exists():
            print(f"drone_sim not built at {SIM_BIN}")
            print("  build: cd planner/cpp/build && cmake --build . --config Release -- -j3")
            return
        if _running_pid("sim"):
            print(f"[sim] already running (pid {_running_pid('sim')})")
            return
        env = dict(os.environ)
        env["DISPLAY"] = SIM_DISPLAY
        env["XAUTHORITY"] = SIM_XAUTH
        log = open(_log_file("sim"), "ab")
        cwd = str(SIM_BIN.parent)
        # stdbuf -oL forces line-buffered stdout so PERF/Replan lines hit the
        # log as soon as they're printed (otherwise C++ block-buffers 4K).
        proc = subprocess.Popen(
            ["stdbuf", "-oL", str(SIM_BIN), str(SIM_CONFIG)],
            stdout=log, stderr=subprocess.STDOUT,
            cwd=cwd, env=env, start_new_session=True,
        )
        _pid_file("sim").write_text(str(proc.pid))
        print(f"[sim] started pid {proc.pid}  (rendering on {SIM_DISPLAY})")
        print(f"       log: {_log_file('sim')}")
        print(f"       window visible via VNC/physical monitor attached to {SIM_DISPLAY}")
    elif args.action == "stop":
        _stop_bg("sim")
    elif args.action == "status":
        pid = _running_pid("sim")
        print(f"sim : {'running pid '+str(pid) if pid else 'stopped'}")
    elif args.action == "logs":
        _tail_log("sim", n=args.n, follow=args.follow)


# ---- mission (phase-1 Python mission server on TCP 18001) -----------------
#
# Binds a TCP Stream Server that DroneStack dials into. Running this alone is
# safe (no HIL access, no motors) — it just serves a protocol. The stabilizer
# binary must be launched separately via `drone.py hw run QD2_DroneStack_PID_2021a`
# for anything to actually move.
def cmd_mission(args):
    if args.action == "start":
        cmd = [sys.executable, str(MISSION_SCRIPT)]
        if args.altitude is not None:
            cmd += ["--altitude", str(args.altitude)]
        if args.hover is not None:
            cmd += ["--hover", str(args.hover)]
        if args.port is not None:
            cmd += ["--port", str(args.port)]
        if args.idle_only:
            cmd += ["--idle-only"]
        _start_bg("mission", cmd)
    elif args.action == "stop":
        _stop_bg("mission")
    elif args.action == "status":
        pid = _running_pid("mission")
        print(f"mission : {'running pid '+str(pid) if pid else 'stopped'}")
    elif args.action == "logs":
        _tail_log("mission", n=args.n, follow=args.follow)


# ---- planner / controller stubs (to be filled when C++ port lands) --------
def cmd_planner(args):
    print("(planner: not yet ported — will live in ./planner/)")


def cmd_control(args):
    print("(control: not yet ported — will live in ./control/)")


# ============================================================================
# argparse
# ============================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog="./drone.py",
        description="QDrone 2 unified management CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="examples:\n"
               "  ./drone.py info\n"
               "  ./drone.py cam start\n"
               "  ./drone.py cam logs -f\n"
               "  ./drone.py hw list\n"
               "  ./drone.py hw run QDrone2_IO_Check_Jan2023\n")
    sub = p.add_subparsers(dest="cmd", metavar="<command>")
    sub.required = True

    sub.add_parser("info", help="system overview + running services").set_defaults(func=cmd_info)

    pc = sub.add_parser("cam", help="RealSense streamer (port 8080)")
    pc.add_argument("action", choices=["start", "stop", "status", "logs"])
    pc.add_argument("-n", type=int, default=50, help="log lines (logs action)")
    pc.add_argument("-f", "--follow", action="store_true", help="follow log")
    pc.set_defaults(func=cmd_cam)

    ph = sub.add_parser("hw", help="QUARC pre-compiled model runner")
    ph.add_argument("action", choices=["list", "run", "stop", "kill"])
    ph.add_argument("model", nargs="?", default="", help="model name from 'hw list'")
    ph.set_defaults(func=cmd_hw)

    pi = sub.add_parser("imu", help="IMU pose publisher (needs sudo)")
    pi.add_argument("action", choices=["start", "stop", "status", "logs"])
    pi.add_argument("-n", type=int, default=50, help="log lines (logs action)")
    pi.add_argument("-f", "--follow", action="store_true", help="follow log")
    pi.set_defaults(func=cmd_imu)

    ps = sub.add_parser("sim", help="CRISP planner drone_sim GLFW app (display :0)")
    ps.add_argument("action", choices=["start", "stop", "status", "logs"])
    ps.add_argument("-n", type=int, default=50, help="log lines (logs action)")
    ps.add_argument("-f", "--follow", action="store_true", help="follow log")
    ps.set_defaults(func=cmd_sim)

    pm = sub.add_parser("mission", help="phase-1 mission server on TCP 18001 (safe alone)")
    pm.add_argument("action", choices=["start", "stop", "status", "logs"])
    pm.add_argument("-n", type=int, default=50, help="log lines (logs action)")
    pm.add_argument("-f", "--follow", action="store_true", help="follow log")
    pm.add_argument("--altitude", type=float, help="hover altitude [m] (start only)")
    pm.add_argument("--hover", type=float, help="hover duration [s] (start only)")
    pm.add_argument("--port", type=int, help=f"TCP port (default {MISSION_PORT}) (start only)")
    pm.add_argument("--idle-only", action="store_true",
                    help="handshake test: stay IDLE forever, no arm (start only)")
    pm.set_defaults(func=cmd_mission)

    sub.add_parser("planner", help="path planner (stub)").set_defaults(func=cmd_planner)
    sub.add_parser("control", help="flight controller (stub)").set_defaults(func=cmd_control)

    return p


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
