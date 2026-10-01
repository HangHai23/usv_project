#!/usr/bin/env python3
"""LAN USB-camera preview and keyboard-controlled YOLO image collection."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import select
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

from aiohttp import web
import cv2


PAGE = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>YOLO 数据采集</title><style>
:root{color-scheme:dark;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}*{box-sizing:border-box}body{margin:0;background:#08090c;color:#f5f5f7}main{max-width:1180px;margin:auto;padding:28px}header{display:flex;align-items:center;justify-content:space-between;gap:18px;margin-bottom:20px}h1{margin:0;font-size:26px;letter-spacing:-.04em}.status{padding:8px 12px;border-radius:99px;background:#25262a;color:#aaa;font-size:12px}.status.on{background:#143a25;color:#62e58b}.viewer{position:relative;overflow:hidden;border:1px solid #292a30;border-radius:24px;background:#111;box-shadow:0 30px 80px #0008}.viewer img{display:block;width:100%;min-height:300px;object-fit:contain;aspect-ratio:16/9}.overlay{position:absolute;left:18px;bottom:18px;padding:8px 11px;border-radius:10px;background:#000a;font-size:12px;backdrop-filter:blur(12px)}.controls{display:grid;grid-template-columns:1fr auto;gap:18px;align-items:center;margin-top:18px;padding:17px 19px;border:1px solid #292a30;border-radius:18px;background:#15161a}.meta{display:flex;flex-wrap:wrap;gap:18px;color:#98989f;font-size:12px}.meta strong{color:#f5f5f7}button{border:0;border-radius:99px;padding:11px 17px;color:white;background:#087bea;font-weight:700;cursor:pointer}button.secondary{background:#303137}.keys{margin-top:14px;color:#85858b;font-size:12px}kbd{padding:3px 7px;border:1px solid #444;border-radius:6px;background:#222;color:white}@media(max-width:650px){main{padding:14px}.controls{grid-template-columns:1fr}.viewer img{min-height:220px}.buttons{display:flex;gap:8px}}
</style></head><body><main><header><div><h1>YOLO 数据采集</h1><small id="camera">正在寻找摄像头…</small></div><span class="status" id="status">未采集</span></header><div class="viewer"><img src="/stream.mjpg" alt="USB摄像头实时画面"><div class="overlay" id="overlay">等待画面</div></div><div class="controls"><div class="meta"><span>已保存 <strong id="count">0</strong> 张</span><span>采集频率 <strong id="fps">—</strong></span><span>目录 <strong id="dir">—</strong></span></div><div class="buttons"><button id="toggle">开始采集</button><button class="secondary" id="single">保存单帧</button></div></div><p class="keys">网页快捷键：<kbd>S</kbd> 开始/暂停　<kbd>C</kbd> 保存单帧。终端另可按 <kbd>Q</kbd> 退出。</p>
<script>const $=id=>document.getElementById(id);let busy=false;async function action(path){if(busy)return;busy=true;try{await fetch(path,{method:'POST'});await update()}finally{busy=false}}async function update(){try{const s=await(await fetch('/api/status',{cache:'no-store'})).json();$('camera').textContent=s.camera||'摄像头未连接';$('count').textContent=s.saved_count;$('fps').textContent=s.capture_fps+' Hz';$('dir').textContent=s.output_dir;$('status').textContent=s.capturing?'正在采集':'未采集';$('status').className='status'+(s.capturing?' on':'');$('toggle').textContent=s.capturing?'暂停采集':'开始采集';$('overlay').textContent=s.frame_size?s.frame_size+' · '+(s.capturing?'REC':'LIVE'):'等待画面'}catch(e){$('camera').textContent='Jetson连接中断'}}$('toggle').onclick=()=>action('/api/toggle');$('single').onclick=()=>action('/api/snapshot');addEventListener('keydown',e=>{if(e.repeat)return;if(e.key.toLowerCase()==='s')action('/api/toggle');if(e.key.toLowerCase()==='c')action('/api/snapshot')});setInterval(update,500);update();</script></main></body></html>"""


class CameraCollector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        # 微秒后缀避免脚本在同一秒内重启时发生目录重名。
        session = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = args.output.expanduser() / "images" / "raw" / session
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._latest_jpeg: bytes | None = None
        self._latest_frame_id = 0
        self._camera_name: str | None = None
        self._frame_size: str | None = None
        self._capturing = False
        self._snapshot_requested = False
        self._saved_count = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._camera_loop, daemon=True)
        metadata = {
            "session": session,
            "created_at": datetime.now().astimezone().isoformat(),
            "image_format": "jpg",
            "width_requested": args.width,
            "height_requested": args.height,
            "camera_fps_requested": args.camera_fps,
            "capture_fps": args.capture_fps,
            "camera_requested": args.camera,
        }
        (self.session_dir / "session.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        self._thread.join(timeout=3)

    def toggle(self) -> bool:
        with self._lock:
            self._capturing = not self._capturing
            return self._capturing

    def snapshot(self) -> None:
        with self._lock:
            self._snapshot_requested = True

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "camera": self._camera_name,
                "frame_size": self._frame_size,
                "capturing": self._capturing,
                "saved_count": self._saved_count,
                "capture_fps": self.args.capture_fps,
                "output_dir": str(self.session_dir),
            }

    def wait_jpeg(self, previous_id: int) -> tuple[int, bytes | None]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._latest_frame_id != previous_id or self._stop.is_set(),
                timeout=2.0,
            )
            return self._latest_frame_id, self._latest_jpeg

    def _candidates(self) -> list[str | int]:
        if self.args.camera != "auto":
            return [int(self.args.camera) if self.args.camera.isdigit() else self.args.camera]
        return [str(path) for path in sorted(Path("/dev").glob("video*"))]

    def _open_camera(self) -> tuple[cv2.VideoCapture | None, str | None]:
        for device in self._candidates():
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                continue
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            capture.set(cv2.CAP_PROP_FPS, self.args.camera_fps)
            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                return capture, str(device)
            capture.release()
        return None, None

    def _save(self, jpeg: bytes) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        with self._lock:
            sequence = self._saved_count + 1
        path = self.session_dir / f"usv_{timestamp}_{sequence:06d}.jpg"
        try:
            path.write_bytes(jpeg)
        except OSError as exc:
            print(f"\n[ERROR] 保存失败: {exc}", flush=True)
            return
        with self._lock:
            self._saved_count = sequence

    def _camera_loop(self) -> None:
        capture: cv2.VideoCapture | None = None
        next_save = 0.0
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality]
        while not self._stop.is_set():
            if capture is None:
                capture, device = self._open_camera()
                if capture is None:
                    with self._lock:
                        self._camera_name = None
                        self._frame_size = None
                    time.sleep(1.0)
                    continue
                with self._lock:
                    self._camera_name = device
                print(f"\n[INFO] 摄像头已连接: {device}", flush=True)
            ok, frame = capture.read()
            if not ok or frame is None:
                print("\n[WARN] 摄像头取帧失败，正在重新扫描…", flush=True)
                capture.release()
                capture = None
                with self._lock:
                    self._camera_name = None
                    self._frame_size = None
                time.sleep(0.5)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue
            jpeg = encoded.tobytes()
            height, width = frame.shape[:2]
            now = time.monotonic()
            with self._condition:
                self._latest_jpeg = jpeg
                self._latest_frame_id += 1
                self._frame_size = f"{width}×{height}"
                capturing = self._capturing
                snapshot = self._snapshot_requested
                self._snapshot_requested = False
                self._condition.notify_all()
            if snapshot or (capturing and now >= next_save):
                self._save(jpeg)
                next_save = now + 1.0 / self.args.capture_fps
            if not capturing:
                next_save = now
        if capture is not None:
            capture.release()


def terminal_keys(collector: CameraCollector, stop_event: threading.Event) -> None:
    if not sys.stdin.isatty():
        return
    descriptor = sys.stdin.fileno()
    original = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        while not stop_event.is_set():
            readable, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not readable:
                continue
            key = os.read(descriptor, 1).decode(errors="ignore").lower()
            if key == "s":
                active = collector.toggle()
                print(f"\n[{'REC' if active else 'PAUSE'}] 连续采集{'开始' if active else '暂停'}", flush=True)
            elif key == "c":
                collector.snapshot()
                print("\n[SNAP] 已请求保存单帧", flush=True)
            elif key == "q":
                stop_event.set()
                break
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)


async def create_app(collector: CameraCollector) -> web.Application:
    app = web.Application()

    async def index(_: web.Request) -> web.Response:
        return web.Response(text=PAGE, content_type="text/html")

    async def status(_: web.Request) -> web.Response:
        return web.json_response(collector.status())

    async def toggle(_: web.Request) -> web.Response:
        return web.json_response({"capturing": collector.toggle()})

    async def snapshot(_: web.Request) -> web.Response:
        collector.snapshot()
        return web.json_response({"accepted": True})

    async def stream(request: web.Request) -> web.StreamResponse:
        response = web.StreamResponse(
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-store",
            }
        )
        await response.prepare(request)
        frame_id = -1
        try:
            while True:
                frame_id, jpeg = await asyncio.to_thread(collector.wait_jpeg, frame_id)
                if jpeg is None:
                    await asyncio.sleep(0.1)
                    continue
                await response.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n" + jpeg + b"\r\n"
                )
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    app.router.add_get("/", index)
    app.router.add_get("/api/status", status)
    app.router.add_post("/api/toggle", toggle)
    app.router.add_post("/api/snapshot", snapshot)
    app.router.add_get("/stream.mjpg", stream)
    return app


async def run(args: argparse.Namespace) -> None:
    collector = CameraCollector(args)
    stop_event = threading.Event()
    collector.start()
    threading.Thread(
        target=terminal_keys, args=(collector, stop_event), daemon=True
    ).start()
    runner = web.AppRunner(await create_app(collector))
    await runner.setup()
    await web.TCPSite(runner, args.bind, args.port).start()
    print(f"[INFO] 网页地址: http://0.0.0.0:{args.port}")
    print(f"[INFO] 保存目录: {collector.session_dir}")
    print("[KEY] S=开始/暂停  C=保存单帧  Q=退出", flush=True)
    try:
        while not stop_event.is_set():
            await asyncio.sleep(0.2)
    finally:
        stop_event.set()
        collector.stop()
        await runner.cleanup()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="auto", help="auto、设备编号或/dev/videoX")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--capture-fps", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--output", type=Path, default=Path.home() / "Downloads" / "usv_yolo_dataset")
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.camera_fps <= 0 or args.capture_fps <= 0:
        parser.error("width, height and frame rates must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("jpeg-quality must be in 1..100")
    if not 1 <= args.port <= 65535:
        parser.error("port must be in 1..65535")
    return args


def main() -> None:
    try:
        asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
