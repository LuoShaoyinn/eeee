#!/usr/bin/env python3
"""Loopback-only Cubie operator dashboard with rectified camera and e-stop."""

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class RobotDashboard:
    def __init__(self, calibration_path: str, socket_path: str, camera: str, preview_file,
                 mission_runner: str, robotbrain: str, protocol_file: str,
                 mission_status_file: str, expected_collectibles: int,
                 localization_file: str, localization_map: str, localization_reset_file: str,
                 initial_x_m: float, initial_y_m: float, initial_yaw_deg: float):
        calibration = cv2.FileStorage(calibration_path, cv2.FILE_STORAGE_READ)
        if not calibration.isOpened():
            raise RuntimeError(f"cannot open calibration: {calibration_path}")
        self.k = calibration.getNode("K").mat()
        self.d = calibration.getNode("D").mat()
        self.rectified_k = calibration.getNode("rectified_K").mat()
        calibration.release()
        if self.k is None or self.d is None or self.rectified_k is None:
            raise RuntimeError("calibration must contain K, D, and rectified_K")
        self.socket_path = socket_path
        self.camera = camera
        self.preview_file = preview_file
        self.mission_runner = mission_runner
        self.robotbrain = robotbrain
        self.protocol_file = Path(protocol_file)
        self.mission_status_file = Path(mission_status_file)
        self.expected_collectibles = expected_collectibles
        self.localization_file = Path(localization_file)
        self.localization_map = Path(localization_map)
        self.localization_reset_file = Path(localization_reset_file)
        self.initial_x_m = initial_x_m
        self.initial_y_m = initial_y_m
        self.initial_yaw_deg = initial_yaw_deg
        self._mission_process = None
        self.e_stop_latched = False
        self.mode = "standby"
        self.last_error = ""
        self._latest_jpeg = b""
        self._frame_time = 0.0
        self._condition = threading.Condition()
        self._running = True

    def robotd(self, command: str) -> str:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(.7)
            client.connect(self.socket_path)
            client.sendall((command + "\n").encode())
            reply = client.recv(1024).decode(errors="replace").strip()
        if reply.startswith("error:"):
            raise RuntimeError(reply)
        return reply

    def emergency_stop(self) -> None:
        self.e_stop_latched = True
        self.mode = "stopped"
        self.stop_auto_collect()
        errors = []
        for command in ("ga25 0", "stop"):
            try:
                self.robotd(command)
            except (OSError, RuntimeError) as error:
                errors.append(str(error))
        if errors:
            self.last_error = "; ".join(errors)

    def release_emergency_stop(self) -> None:
        """Unlock the software latch without commanding any motion."""
        errors = []
        for command in ("ga25 0", "stop"):
            try:
                self.robotd(command)
            except (OSError, RuntimeError) as error:
                errors.append(str(error))
        if errors:
            self.last_error = "; ".join(errors)
            return
        self.e_stop_latched = False
        self.mode = "standby"
        self.last_error = ""

    def start_preview(self) -> None:
        if not self.e_stop_latched:
            self.mode = "decision_preview"

    def stop_auto_collect(self) -> None:
        if self._mission_process is None or self._mission_process.poll() is not None:
            return
        self._mission_process.terminate()
        try:
            self._mission_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._mission_process.kill()
            self._mission_process.wait(timeout=2)

    def start_mission(self, mode: str) -> None:
        """Launch the supervised V1 mission only after the operator clicks it."""
        if self.e_stop_latched:
            self.last_error = "release the emergency-stop lock before auto collection"
            return
        if self._mission_process is not None and self._mission_process.poll() is None:
            self.mode = mode
            return
        try:
            age = time.time() - self.protocol_file.stat().st_mtime
            if age > 1.5:
                raise RuntimeError(f"vision/localization frame is stale ({age:.2f}s)")
            self.robotd("ga25 0")
            self.robotd("stop")
            self._mission_process = subprocess.Popen(
                [sys.executable, self.mission_runner, "--robotbrain", self.robotbrain,
                 "--protocol-file", str(self.protocol_file), "--status-file", str(self.mission_status_file),
                 "--expected-objects", str(self.expected_collectibles)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.mode = mode
            self.last_error = ""
        except (OSError, RuntimeError) as error:
            self._mission_process = None
            self.mode = "standby"
            self.last_error = str(error)

    def start_auto_collect(self) -> None:
        self.start_mission("auto_collect")

    def start_collect_and_home(self) -> None:
        self.start_mission("collect_and_home")

    def mission_status(self) -> dict:
        status = {"running": False, "state": "idle", "error": ""}
        # A status file can survive a dashboard restart or a power cycle. It
        # is authoritative only while this dashboard owns a live runner.
        if self._mission_process is None:
            return status
        try:
            status.update(json.loads(self.mission_status_file.read_text(encoding="utf-8")))
        except FileNotFoundError:
            pass
        except (OSError, json.JSONDecodeError) as error:
            status["error"] = str(error)
        if self._mission_process is not None and self._mission_process.poll() is not None and \
                self.mode in {"auto_collect", "collect_and_home"}:
            self.mode = "mission_finished" if not status.get("error") else "mission_fault"
        return status

    def localization_status(self) -> dict:
        try:
            return json.loads(self.localization_file.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {"valid": False, "source": "waiting for vision bridge"}
        except (OSError, json.JSONDecodeError) as error:
            return {"valid": False, "source": "unavailable", "error": str(error)}

    def reset_localization(self) -> None:
        """Reset only the displayed V1 pose reference; this never commands motors."""
        try:
            self.localization_reset_file.parent.mkdir(parents=True, exist_ok=True)
            descriptor, temporary = tempfile.mkstemp(prefix=".robot-localization-reset-",
                                                     dir=self.localization_reset_file.parent, text=True)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as output:
                    json.dump({"x_m": self.initial_x_m, "y_m": self.initial_y_m,
                               "yaw_deg": self.initial_yaw_deg}, output)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(temporary, self.localization_reset_file)
            except Exception:
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass
                raise
            self.last_error = ""
        except OSError as error:
            self.last_error = str(error)

    def status(self) -> dict:
        state = "unavailable"
        try:
            state = self.robotd("state")
        except (OSError, RuntimeError) as error:
            self.last_error = str(error)
        return {
            "e_stop_latched": self.e_stop_latched,
            "mode": self.mode,
            "frame_age_ms": round((time.monotonic() - self._frame_time) * 1000) if self._frame_time else None,
            "robot_state": state,
            "mission": self.mission_status(),
            "localization": self.localization_status(),
            "error": self.last_error,
        }

    def camera_loop(self) -> None:
        if self.preview_file:
            self.preview_file_loop()
            return
        capture = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.set(cv2.CAP_PROP_FPS, 20)
        if not capture.isOpened():
            self.last_error = f"cannot open camera: {self.camera}"
            return
        map_size = None
        map_x = map_y = None
        while self._running:
            ok, raw = capture.read()
            if not ok or raw is None:
                self.last_error = "camera capture failed"
                time.sleep(.1)
                continue
            if map_size != raw.shape[:2]:
                height, width = raw.shape[:2]
                sx, sy = width / 1280.0, height / 720.0
                source_k = self.k.astype(np.float64).copy()
                output_k = self.rectified_k.astype(np.float64).copy()
                for matrix in (source_k, output_k):
                    matrix[0, 0] *= sx
                    matrix[0, 2] *= sx
                    matrix[1, 1] *= sy
                    matrix[1, 2] *= sy
                map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
                    source_k, self.d, np.eye(3), output_k, (width, height), cv2.CV_16SC2)
                map_size = raw.shape[:2]
            preview = cv2.remap(raw, map_x, map_y, cv2.INTER_LINEAR)
            label = "E-STOP LATCHED" if self.e_stop_latched else "RECTIFIED PREVIEW"
            color = (20, 20, 230) if self.e_stop_latched else (20, 190, 20)
            cv2.rectangle(preview, (12, 12), (340, 54), (15, 15, 15), -1)
            cv2.putText(preview, label, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, .78, color, 2)
            ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                self.last_error = "cannot encode camera preview"
                continue
            with self._condition:
                self._latest_jpeg = encoded.tobytes()
                self._frame_time = time.monotonic()
                self._condition.notify_all()
        capture.release()

    def preview_file_loop(self) -> None:
        while self._running:
            preview = cv2.imread(self.preview_file, cv2.IMREAD_COLOR)
            if preview is None:
                self.last_error = f"waiting for vision frame: {self.preview_file}"
                time.sleep(.1)
                continue
            # The bridge may start after the dashboard.  Once a valid preview
            # arrives, discard only that transient status; do not hide real
            # hardware or emergency-stop errors.
            if self.last_error.startswith("waiting for vision frame:"):
                self.last_error = ""
            label = "E-STOP LATCHED" if self.e_stop_latched else "RECTIFIED VISION PREVIEW"
            color = (20, 20, 230) if self.e_stop_latched else (20, 190, 20)
            cv2.rectangle(preview, (12, 12), (440, 54), (15, 15, 15), -1)
            cv2.putText(preview, label, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, .72, color, 2)
            ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                with self._condition:
                    self._latest_jpeg = encoded.tobytes()
                    self._frame_time = time.monotonic()
                    self._condition.notify_all()
            else:
                self.last_error = "cannot encode vision preview"
            time.sleep(.08)

    def next_frame(self, previous: bytes) -> bytes:
        with self._condition:
            self._condition.wait_for(lambda: self._latest_jpeg != previous or not self._running, timeout=1.0)
            return self._latest_jpeg


PAGE = """<!doctype html><html lang='zh-CN'><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'><title>Cubie 控制台</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#081116;color:#e7f0ef;font:16px system-ui,sans-serif}
main{max-width:1280px;margin:auto;padding:20px;display:grid;gap:16px;grid-template-columns:minmax(0,2fr) minmax(270px,1fr)}
h1{grid-column:1/-1;font-size:1.35rem;margin:0;color:#80e6b5}.camera,.map{background:#101c21;border:1px solid #25434b;border-radius:12px;overflow:hidden}.camera img,.map img{display:block;width:100%;min-height:300px;object-fit:contain;background:#000}.map h2{margin:12px 16px;font-size:1rem}.panel{background:#101c21;border:1px solid #25434b;border-radius:12px;padding:18px}.state{white-space:pre-wrap;word-break:break-word;font:13px ui-monospace,monospace;color:#b8d8d4}.tag{display:inline-block;padding:5px 9px;border-radius:999px;background:#1f4d40;color:#9bf0c5;font-weight:700}.fault{background:#631d27;color:#ffb9bd}button{width:100%;margin-top:16px;padding:18px;border:0;border-radius:10px;background:#d93543;color:white;font-weight:800;font-size:1.2rem;cursor:pointer}.start,.collect{background:#16885c}.collect{background:#2463c9}button:active{transform:scale(.98)}.note{font-size:.87rem;color:#a4bbb8;line-height:1.5}
@media(max-width:780px){main{grid-template-columns:1fr;padding:12px}.camera img{min-height:220px}}
</style><main><h1>Cubie 整机控制台</h1><section class='camera'><img src='/stream.mjpg' alt='校正后的实时相机画面与 YOLO 检测框'></section><section class='map'><h2>V1 场地定位图</h2><img id='localization-map' src='/localization.png' alt='蓝色围栏定位得到的场地位置图'></section><section class='panel'><span id='tag' class='tag'>正在连接</span><h2>整机状态</h2><div id='state' class='state'>读取中…</div><button id='start' class='start'>开始决策预览</button><button id='collect' class='collect'>开始自动收集（黄/红）</button><button id='collect-home' class='collect'>收集完黄/红后返航 Home</button><button id='reset-localization'>定位复位到起点</button><button id='release'>解除急停锁定</button><button id='stop'>停止并急停</button><p class='note'>返航任务仅在 yellow 与 red 均确认收集后寻找 home 并导航靠近；未配置翻板脉冲。定位复位只重置地图参考点，不驱动车体。</p></section></main><script>
async function status(){try{const r=await fetch('/api/status');const s=await r.json();const m=s.mission||{},l=s.localization||{};document.querySelector('#tag').textContent=s.e_stop_latched?'急停已锁定':s.mode==='collect_and_home'?'收集返航任务中':s.mode==='auto_collect'?'自动收集中':s.mode==='decision_preview'?'决策预览中':'系统就绪';document.querySelector('#tag').className='tag '+(s.e_stop_latched?'fault':'');document.querySelector('#state').textContent='模式: '+s.mode+'\\n图像延迟: '+(s.frame_age_ms??'—')+' ms\\n定位: '+(l.valid?'有效':'无效')+(l.x_m!==undefined?'  x='+l.x_m+'m y='+l.y_m+'m':'')+'\\n任务: '+(m.state??'idle')+(m.error?'\\n任务错误: '+m.error:'')+'\\n'+s.robot_state+(s.error?'\\n错误: '+s.error:'');document.querySelector('#localization-map').src='/localization.png?t='+Date.now()}catch(e){document.querySelector('#state').textContent='状态读取失败: '+e}}document.querySelector('#start').onclick=async()=>{await fetch('/api/start',{method:'POST'});status()};document.querySelector('#collect').onclick=async()=>{await fetch('/api/auto-collect',{method:'POST'});status()};document.querySelector('#collect-home').onclick=async()=>{await fetch('/api/collect-and-home',{method:'POST'});status()};document.querySelector('#reset-localization').onclick=async()=>{await fetch('/api/reset-localization',{method:'POST'});status()};document.querySelector('#release').onclick=async()=>{await fetch('/api/release-estop',{method:'POST'});status()};document.querySelector('#stop').onclick=async()=>{await fetch('/api/estop',{method:'POST'});status()};status();setInterval(status,500)
</script></html>"""


def handler_factory(dashboard: RobotDashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            return

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.end_headers()
                self.wfile.write(PAGE.encode())
            elif path == "/api/status":
                payload = json.dumps(dashboard.status()).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
            elif path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                previous = b""
                try:
                    while dashboard._running:
                        frame = dashboard.next_frame(previous)
                        if not frame:
                            continue
                        previous = frame
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif path == "/localization.png":
                try:
                    payload = dashboard.localization_map.read_bytes()
                except FileNotFoundError:
                    self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "waiting for localization map")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            path = urlsplit(self.path).path
            if path == "/api/estop":
                dashboard.emergency_stop()
            elif path == "/api/release-estop":
                dashboard.release_emergency_stop()
            elif path == "/api/start":
                dashboard.start_preview()
            elif path == "/api/auto-collect":
                dashboard.start_auto_collect()
            elif path == "/api/collect-and-home":
                dashboard.start_collect_and_home()
            elif path == "/api/reset-localization":
                dashboard.reset_localization()
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self.send_response(HTTPStatus.NO_CONTENT)
            self.end_headers()
    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="keep 127.0.0.1 unless using a secured network")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--socket", default="/tmp/robotd.sock")
    parser.add_argument("--calibration", default="config/camera_fisheye_1280x720.yaml")
    parser.add_argument("--preview-file", help="use a rectified vision frame written by robotvision_bridge")
    parser.add_argument("--mission-runner", default="tools/live_mission.py")
    parser.add_argument("--robotbrain", default="build/robotbrain")
    parser.add_argument("--protocol-file", default="/tmp/robotvision-frame.txt")
    parser.add_argument("--mission-status-file", default="/tmp/robot-mission-status.json")
    parser.add_argument("--expected-collectibles", type=int, default=2)
    parser.add_argument("--localization-file", default="/tmp/robot-localization.json")
    parser.add_argument("--localization-map", default="/tmp/robot-localization-map.png")
    parser.add_argument("--localization-reset-file", default="/tmp/robot-localization-reset.json")
    parser.add_argument("--initial-x", type=float, default=.10)
    parser.add_argument("--initial-y", type=float, default=.10)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    args = parser.parse_args()
    dashboard = RobotDashboard(args.calibration, args.socket, args.camera, args.preview_file,
                               args.mission_runner, args.robotbrain, args.protocol_file,
                               args.mission_status_file, args.expected_collectibles,
                               args.localization_file, args.localization_map, args.localization_reset_file,
                               args.initial_x, args.initial_y, args.initial_yaw)
    threading.Thread(target=dashboard.camera_loop, daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(dashboard))
    print(f"dashboard: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard._running = False
        dashboard.emergency_stop()
        server.server_close()


if __name__ == "__main__":
    main()
