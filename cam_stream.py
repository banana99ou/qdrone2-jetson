#!/usr/bin/env python3
"""MJPEG + PointCloud + Pose streamer for RealSense D435 + QDrone IMU.
Endpoints:
  /          - index (color + depth side by side)
  /color     - MJPEG of color frames
  /depth     - MJPEG of colorized depth frames
  /snapshot  - single JPEG (color + depth stacked)
  /pc        - HTML page, live 3D point cloud (Three.js)
  /points    - binary blob: uint32 N, then N*xyz(float32), then N*rgb(uint8)
  /pose      - JSON {ok, age, x, y, z, qx, qy, qz, qw} from imu_service over UDP
"""
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np
from pyrealsense2 import pyrealsense2 as rs


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


PORT = 8080
POSE_UDP_PORT = 8090
W, H, FPS = 640, 480, 30
PC_STRIDE = 4          # every 4 px  -> 160x120 = 19200 points before filtering
PC_MAX_RANGE_M = 6.0   # clip far points

latest = {"color": None, "depth": None, "points": None, "ts": 0.0}
lock = threading.Lock()

# body->world quaternion (wxyz) + world position; ts is monotonic recv time
pose_state = {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
              "x": 0.0, "y": 0.0, "z": 0.0, "ts": 0.0}
pose_lock = threading.Lock()


def pose_rx_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", POSE_UDP_PORT))
    while True:
        try:
            data, _ = s.recvfrom(128)
        except OSError:
            continue
        if len(data) < 36:
            continue
        host_ts, qw, qx, qy, qz, px, py, pz = struct.unpack("<d4f3f", data[:36])
        with pose_lock:
            pose_state["qw"] = qw
            pose_state["qx"] = qx
            pose_state["qy"] = qy
            pose_state["qz"] = qz
            pose_state["x"] = px
            pose_state["y"] = py
            pose_state["z"] = pz
            pose_state["ts"] = time.monotonic()


def capture_loop():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    pipe.start(cfg)
    colorizer = rs.colorizer()
    align = rs.align(rs.stream.color)
    pc = rs.pointcloud()
    try:
        while True:
            frames = pipe.wait_for_frames()
            frames = align.process(frames)
            c = frames.get_color_frame()
            d = frames.get_depth_frame()
            if not c or not d:
                continue
            color_img = np.asanyarray(c.get_data())
            depth_vis = np.asanyarray(colorizer.colorize(d).get_data())
            _, cj = cv2.imencode(".jpg", color_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            _, dj = cv2.imencode(".jpg", depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 80])

            points = pc.calculate(d)
            verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(H, W, 3)
            v_ds = verts[::PC_STRIDE, ::PC_STRIDE]
            rgb_ds = color_img[::PC_STRIDE, ::PC_STRIDE]
            # RealSense optical frame: (+x right, +y down, +z forward)
            # valid filter is applied in the *camera* depth (cam_z)
            cam_z = v_ds[..., 2]
            valid = (cam_z > 0.05) & (cam_z < PC_MAX_RANGE_M)
            cam_xyz = v_ds[valid]
            # remap to drone body frame (+x forward, +y right, +z up)
            xyz = np.empty_like(cam_xyz)
            xyz[:, 0] =  cam_xyz[:, 2]   # fwd  =  cam_z
            xyz[:, 1] =  cam_xyz[:, 0]   # right=  cam_x
            xyz[:, 2] = -cam_xyz[:, 1]   # up   = -cam_y
            xyz = xyz.astype(np.float32)
            bgr = rgb_ds[valid]
            rgb = bgr[:, ::-1].astype(np.uint8)
            n = xyz.shape[0]
            header = struct.pack("<I", n)
            blob = header + xyz.tobytes() + rgb.tobytes()

            with lock:
                latest["color"] = cj.tobytes()
                latest["depth"] = dj.tobytes()
                latest["points"] = blob
                latest["ts"] = time.time()
    finally:
        pipe.stop()


INDEX = b"""<!DOCTYPE html><html><head><title>QDrone D435</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:12px}
h2{margin:6px}.row{display:flex;gap:8px;flex-wrap:wrap}
.row img{max-width:48vw;border:1px solid #444}
a{color:#6cf}</style></head>
<body><h2>QDrone 2 / RealSense D435 &mdash; live</h2>
<p><a href="/pc">&rarr; 3D point cloud view</a></p>
<div class="row">
  <div><h3>Color</h3><img src="/color"></div>
  <div><h3>Depth (colorized)</h3><img src="/depth"></div>
</div></body></html>"""


PC_HTML = """<!DOCTYPE html><html><head><title>D435 PointCloud</title>
<style>html,body{margin:0;height:100%;background:#111;color:#ccc;font-family:sans-serif;overflow:hidden}
#info{position:fixed;top:8px;left:8px;z-index:10;background:#000c;padding:8px 12px;border-radius:4px;font-size:12px;line-height:1.5}
#info a{color:#6cf}
#info b.x{color:#ff4040}#info b.y{color:#40e040}#info b.z{color:#4080ff}
canvas{display:block}</style>
<script type="importmap">{"imports":{
  "three":"https://unpkg.com/three@0.160.0/build/three.module.js",
  "three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"
}}</script></head>
<body><div id="info">
<a href="/">&larr; 2D views</a> &nbsp;|&nbsp; drag=rotate &nbsp; wheel=zoom &nbsp; right-drag=pan<br>
body frame &nbsp; <b class="x">+X forward</b> &nbsp; <b class="y">+Y right</b> &nbsp; <b class="z">+Z up</b> &nbsp; (grid = 1 m)<br>
<span id="stat">loading&hellip;</span>
</div>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);

// world: Z up. pointcloud is already in drone body frame (x=fwd, y=right, z=up).
const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.01, 200);
camera.up.set(0, 0, 1);
camera.position.set(-3, -3, 2);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(1.5, 0, 0);   // look roughly forward of origin
controls.update();

// ---- helpers ----
function axisLine(end, color){
  const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0), end]);
  return new THREE.Line(g, new THREE.LineBasicMaterial({color, linewidth:2}));
}

function makeLabel(text, color){
  const canvas = document.createElement('canvas');
  canvas.width = 128; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.font = 'bold 48px sans-serif';
  ctx.fillStyle = color;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 32);
  const tex = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({map: tex, depthTest:false, transparent:true});
  const s = new THREE.Sprite(mat);
  s.scale.set(0.3, 0.15, 1);
  return s;
}
// ---- grid on XY plane (z=0), 1 m cells, 20x20 m  (world frame) ----
const GRID_SIZE = 20, GRID_DIV = 20;
const grid = new THREE.GridHelper(GRID_SIZE, GRID_DIV, 0x888888, 0x333333);
grid.rotation.x = Math.PI/2;    // default GridHelper is XZ-plane; rotate onto XY
scene.add(grid);

// tick-labels every 1 m along +X and +Y, up to 5 m
for (let i = 1; i <= 5; i++){
  const lxi = makeLabel(i+' m', '#ff8080');
  lxi.position.set(i, -0.25, 0.02); lxi.scale.set(0.25, 0.12, 1); scene.add(lxi);
  const lyi = makeLabel(i+' m', '#80ff80');
  lyi.position.set(-0.25, i, 0.02); lyi.scale.set(0.25, 0.12, 1); scene.add(lyi);
}

// ---- droneFrame: everything below rotates/translates with the drone pose ----
const droneFrame = new THREE.Group();
scene.add(droneFrame);

// body axes (1 m) + labels inside droneFrame
droneFrame.add(axisLine(new THREE.Vector3(1,0,0), 0xff4040));
droneFrame.add(axisLine(new THREE.Vector3(0,1,0), 0x40e040));
droneFrame.add(axisLine(new THREE.Vector3(0,0,1), 0x4080ff));
const lx = makeLabel('X (fwd)',   '#ff4040'); lx.position.set(1.15, 0, 0.05); droneFrame.add(lx);
const ly = makeLabel('Y (right)', '#40e040'); ly.position.set(0, 1.15, 0.05); droneFrame.add(ly);
const lz = makeLabel('Z (up)',    '#4080ff'); lz.position.set(0, 0, 1.15);    droneFrame.add(lz);

// drone body marker (wireframe box 0.3m)
const body = new THREE.Mesh(
  new THREE.BoxGeometry(0.3, 0.3, 0.1),
  new THREE.MeshBasicMaterial({color:0xffcc00, wireframe:true})
);
droneFrame.add(body);

// point cloud (points are already in drone body frame from server)
const geom = new THREE.BufferGeometry();
const MAX_POINTS = 200000;
geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS*3), 3));
geom.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(MAX_POINTS*3), 3));
geom.setDrawRange(0, 0);
const mat = new THREE.PointsMaterial({size:0.01, vertexColors:true, sizeAttenuation:true});
const cloud = new THREE.Points(geom, mat);
droneFrame.add(cloud);

addEventListener('resize', () => {
  camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

const stat = document.getElementById('stat');
let fps = 0, frames = 0, last = performance.now();
let poseStr = 'pose: offline';

async function fetchCloud(){
  try{
    const r = await fetch('/points', {cache:'no-store'});
    const buf = await r.arrayBuffer();
    const n = new DataView(buf).getUint32(0, true);
    const xyz = new Float32Array(buf, 4, n*3);
    const rgb = new Uint8Array(buf, 4 + n*12, n*3);
    const pos = geom.attributes.position.array;
    const col = geom.attributes.color.array;
    const m = Math.min(n, MAX_POINTS);
    pos.set(xyz.subarray(0, m*3));
    for (let i=0;i<m*3;i++) col[i] = rgb[i]/255;
    geom.attributes.position.needsUpdate = true;
    geom.attributes.color.needsUpdate = true;
    geom.setDrawRange(0, m);
    geom.computeBoundingSphere();
    frames++;
    const now = performance.now();
    if (now - last > 1000){ fps = frames*1000/(now-last); frames = 0; last = now; }
    stat.textContent = `${m.toLocaleString()} pts  |  ${fps.toFixed(1)} Hz  |  ${poseStr}`;
  }catch(e){ stat.textContent = 'fetch err: '+e.message; }
}
setInterval(fetchCloud, 100);   // ~10 Hz

async function fetchPose(){
  try{
    const r = await fetch('/pose', {cache:'no-store'});
    const p = await r.json();
    if (p.ok){
      droneFrame.position.set(p.x, p.y, p.z);
      droneFrame.quaternion.set(p.qx, p.qy, p.qz, p.qw);
      const rpy = quatToRPY(p.qw, p.qx, p.qy, p.qz);
      poseStr = `rpy ${rpy[0].toFixed(1)}° ${rpy[1].toFixed(1)}° ${rpy[2].toFixed(1)}°  (age ${p.age.toFixed(2)}s)`;
    } else {
      poseStr = 'pose: offline (run: sudo ./drone.py imu start)';
      droneFrame.position.set(0,0,0);
      droneFrame.quaternion.set(0,0,0,1);
    }
  }catch(e){ poseStr = 'pose err: '+e.message; }
}
function quatToRPY(w,x,y,z){
  const sinr = 2*(w*x + y*z), cosr = 1 - 2*(x*x + y*y);
  const roll = Math.atan2(sinr, cosr);
  const sinp = 2*(w*y - z*x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp)*Math.PI/2 : Math.asin(sinp);
  const siny = 2*(w*z + x*y), cosy = 1 - 2*(y*y + z*z);
  const yaw = Math.atan2(siny, cosy);
  return [roll*180/Math.PI, pitch*180/Math.PI, yaw*180/Math.PI];
}
setInterval(fetchPose, 33);    // ~30 Hz

function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
animate();
</script></body></html>""".encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _mjpeg(self, key):
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        last = 0.0
        try:
            while True:
                with lock:
                    buf, ts = latest[key], latest["ts"]
                if buf is None or ts == last:
                    time.sleep(0.005)
                    continue
                last = ts
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode())
                self.wfile.write(buf)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX)
        elif self.path == "/pc":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PC_HTML)
        elif self.path == "/color":
            self._mjpeg("color")
        elif self.path == "/depth":
            self._mjpeg("depth")
        elif self.path == "/points":
            with lock:
                blob = latest["points"]
            if blob is None:
                self.send_response(503); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            try:
                self.wfile.write(blob)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path == "/pose":
            with pose_lock:
                p = dict(pose_state)
            age = time.monotonic() - p["ts"] if p["ts"] > 0 else float("inf")
            ok = age < 1.0
            # publish as xyzw for Three.js convenience
            out = {
                "ok": ok,
                "age": min(age, 999.0),
                "x": p["x"], "y": p["y"], "z": p["z"],
                "qx": p["qx"], "qy": p["qy"], "qz": p["qz"], "qw": p["qw"],
            }
            body_bytes = json.dumps(out).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body_bytes)))
            self.end_headers()
            try:
                self.wfile.write(body_bytes)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path == "/snapshot":
            with lock:
                c, d = latest["color"], latest["depth"]
            if c is None:
                self.send_response(503); self.end_headers(); return
            c_img = cv2.imdecode(np.frombuffer(c, np.uint8), cv2.IMREAD_COLOR)
            d_img = cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
            stacked = np.hstack([c_img, d_img])
            _, j = cv2.imencode(".jpg", stacked, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(j)))
            self.end_headers()
            self.wfile.write(j.tobytes())
        else:
            self.send_response(404); self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=capture_loop, daemon=True).start()
    threading.Thread(target=pose_rx_loop, daemon=True).start()
    while latest["color"] is None:
        time.sleep(0.1)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Serving on 0.0.0.0:{PORT}")
    print(f"  http://<host>:{PORT}/     -> color + depth")
    print(f"  http://<host>:{PORT}/pc   -> 3D point cloud (with pose overlay)")
    print(f"  pose UDP in: 127.0.0.1:{POSE_UDP_PORT}")
    srv.serve_forever()
