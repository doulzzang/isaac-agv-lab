"""Localhost-only browser control bridge for Interactive Physics Lab."""

from __future__ import annotations

import json
import uuid
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote


ALLOWED_ACTIONS = {
    "message_ack", "navigation_select", "navigation_refresh", "navigation_goal",
    "room_change", "room_reset", "room_discard", "room_save", "room_load", "room_refresh",
    "object_select", "object_add", "object_apply", "object_delete",
    "view_only", "view_panels", "app_exit",
    "scene_minimal", "scene_warehouse", "scene_warehouse_dongshin", "add_agv",
    "worker_start", "worker_stop", "worker_reset", "worker_rotate",
    "worker_smaller", "worker_larger",
    "humidity_prepare", "humidity_up", "humidity_down",
    "temperature_prepare", "temperature_up", "temperature_down",
    "robot_forward", "robot_backward", "robot_left", "robot_right",
    "robot_release", "robot_stop", "robot_reset", "robot_resize_default", "robot_body_red",
    "prepare_next_experience", "run_experience",
    "agv_camera", "close_camera", "view_overview",
    "view_pan_left", "view_pan_right", "view_pan_up", "view_pan_down",
    "view_zoom_in", "view_zoom_out", "view_fullscreen", "view_restore",
    "fixed", "movable", "kinematic", "mass", "smaller", "larger",
    "color", "model", "agent_prompt",
    "face_select_uploaded", "face_map_2d",
}


def extract_asset_zip(content, destination):
    import io,zipfile,stat,shutil
    from pathlib import PurePosixPath
    destination=Path(destination)
    temporary=destination.with_name(destination.name+'.pending')
    if destination.exists():return
    try:
        with zipfile.ZipFile(io.BytesIO(content)) as archive:
            entries=archive.infolist()
            if len(entries)>1000 or sum(item.file_size for item in entries)>100*1024*1024:
                raise ValueError('ZIP은 압축 해제 후 100MB, 1000개 파일 이하만 지원합니다.')
            names=set();has_usd=False
            for item in entries:
                path=PurePosixPath(item.filename)
                if not item.filename or path.is_absolute() or '..' in path.parts or '\\' in item.filename or ':' in item.filename or stat.S_ISLNK(item.external_attr>>16) or item.filename in names:
                    raise ValueError('ZIP에 안전하지 않거나 중복된 경로가 있습니다.')
                names.add(item.filename)
                has_usd=has_usd or (not item.is_dir() and path.suffix.lower() in ('.usd','.usda','.usdc','.usdz'))
            if not has_usd:raise ValueError('ZIP 안에 USD 공간/모델 파일이 없습니다.')
            temporary.mkdir(parents=True,exist_ok=False)
            for item in entries:
                target=temporary/item.filename
                if item.is_dir():target.mkdir(parents=True,exist_ok=True);continue
                target.parent.mkdir(parents=True,exist_ok=True)
                with archive.open(item) as source,target.open('xb') as output:shutil.copyfileobj(source,output)
        temporary.replace(destination)
    except Exception:
        if temporary.exists():shutil.rmtree(temporary)
        raise



class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class ExternalControlServer:
    """Serve one HTML panel and queue validated commands for Kit's main loop."""

    def __init__(self, html_path: Path, host="127.0.0.1", first_port=8765):
        self._html_path = Path(html_path)
        self._face_upload_dir = self._html_path.parent / "uploads"
        self._host = host
        self._first_port = int(first_port)
        self._commands = queue.Queue(maxsize=128)
        self._status_lock = threading.Lock()
        self._status = {"connection": "waiting for Isaac Sim status"}
        self._server = None
        self._thread = None
        self._camera_frame_path = None

    def set_camera_frame_path(self, path):
        """Publish the latest atomically-written JPEG to browser clients."""
        self._camera_frame_path = None if path is None else Path(path)

    @property
    def url(self):
        if self._server is None:
            return None
        return f"http://{self._host}:{self._server.server_port}/"

    def start(self):
        if self._server is not None:
            return self.url
        if not self._html_path.is_file():
            raise FileNotFoundError(self._html_path)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "IsaacLabLocal/1.0"

            def log_message(self, _format, *_args):
                return

            def _send(self, status, content_type, payload):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'")
                self.end_headers()
                self.wfile.write(payload)

            def do_GET(self):
                if self.path.startswith("/camera.jpg"):
                    frame = owner._camera_frame_path
                    if frame is None or not frame.is_file():
                        self._send(503, "text/plain; charset=utf-8", b"camera not ready")
                        return
                    try:
                        payload = frame.read_bytes()
                    except OSError:
                        self._send(503, "text/plain; charset=utf-8", b"camera frame unavailable")
                        return
                    self._send(200, "image/jpeg", payload)
                    return
                if self.path == "/status":
                    with owner._status_lock:
                        payload = json.dumps(
                            owner._status, ensure_ascii=False, separators=(",", ":")
                        ).encode("utf-8")
                    self._send(200, "application/json; charset=utf-8", payload)
                    return
                if self.path not in {"/", "/index.html"}:
                    self._send(404, "text/plain; charset=utf-8", b"Not found")
                    return
                self._send(
                    200,
                    "text/html; charset=utf-8",
                    owner._html_path.read_bytes(),
                )

            def do_POST(self):
                if self.path == "/asset-upload":
                    try:
                        origin = self.headers.get("Origin")
                        if origin and origin != "http://" + self.headers.get("Host", ""):
                            raise ValueError("origin not allowed")
                        length = int(self.headers.get("Content-Length", "0"))
                        name = Path(unquote(self.headers.get("X-File-Name", ""))).name
                        suffix = Path(name).suffix.lower()
                        if not 0 < length <= 20 * 1024 * 1024 or suffix not in {".usd", ".usda", ".usdc", ".usdz", ".zip"}:
                            raise ValueError("20MB 이하 USD/USDA/USDC/USDZ/ZIP 파일을 선택하세요.")
                        content = self.rfile.read(length)
                        if len(content) != length:
                            raise ValueError("incomplete upload")
                        purpose = self.headers.get("X-Asset-Kind", "room")
                        if purpose not in {"room", "object"}:
                            raise ValueError("잘못된 파일 용도")
                        folder = owner._html_path.parents[4] / ".web_runtime" / (purpose + "_assets")
                        folder.mkdir(parents=True, exist_ok=True)
                        identity = str(uuid.uuid4()) + suffix
                        destination = folder / identity
                        if suffix == ".zip":
                            extract_asset_zip(content, destination)
                        else:
                            temporary = destination.with_name(identity + ".pending")
                            temporary.write_bytes(content)
                            temporary.replace(destination)
                            destination.with_suffix(suffix + ".json").write_text(json.dumps({"label": Path(name).stem}, ensure_ascii=False), encoding="utf-8")
                        owner._commands.put_nowait(("room_refresh", None))
                        payload = json.dumps({"id": identity, "message": "운영 PC에 저장됨 · 목록 갱신 중…"}, ensure_ascii=False).encode("utf-8")
                        self._send(200, "application/json; charset=utf-8", payload)
                    except Exception as error:
                        self._send(400, "application/json; charset=utf-8", json.dumps({"error": str(error)}, ensure_ascii=False).encode("utf-8"))
                    return
                if self.path == "/face-upload":
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if length <= 0 or length > 15 * 1024 * 1024:
                            raise ValueError("face image must be 1 byte to 15 MiB")
                        original_name = Path(
                            unquote(self.headers.get("X-File-Name", "face.jpg"))
                        ).name
                        suffix = Path(original_name).suffix.lower()
                        if suffix not in {".jpg", ".jpeg", ".png"}:
                            raise ValueError("only JPG, JPEG and PNG are allowed")
                        owner._face_upload_dir.mkdir(parents=True, exist_ok=True)
                        destination = owner._face_upload_dir / (
                            f"face_{time.time_ns()}{suffix}"
                        )
                        destination.write_bytes(self.rfile.read(length))
                        if destination.stat().st_size != length:
                            destination.unlink(missing_ok=True)
                            raise ValueError("incomplete upload")
                        owner._commands.put_nowait(
                            ("face_select_uploaded", str(destination.resolve()))
                        )
                        payload = json.dumps(
                            {"ok": True, "name": original_name},
                            ensure_ascii=False,
                        ).encode("utf-8")
                        self._send(200, "application/json; charset=utf-8", payload)
                    except queue.Full:
                        self._send(429, "application/json", b'{"ok":false,"error":"queue full"}')
                    except Exception as error:
                        payload = json.dumps(
                            {"ok": False, "error": str(error)},
                            ensure_ascii=False,
                        ).encode("utf-8")
                        self._send(400, "application/json; charset=utf-8", payload)
                    return
                if self.path != "/command":
                    self._send(404, "application/json", b'{"ok":false}')
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if length <= 0 or length > 8192:
                        raise ValueError("invalid content length")
                    data = json.loads(self.rfile.read(length).decode("utf-8"))
                    action = str(data.get("action", ""))
                    value = data.get("value")
                    if action not in ALLOWED_ACTIONS:
                        raise ValueError("unknown action")
                    owner._commands.put_nowait((action, value))
                    self._send(200, "application/json", b'{"ok":true}')
                except queue.Full:
                    self._send(429, "application/json", b'{"ok":false,"error":"queue full"}')
                except Exception:
                    self._send(400, "application/json", b'{"ok":false,"error":"bad request"}')

        last_error = None
        for port in range(self._first_port, self._first_port + 11):
            try:
                self._server = _ReusableThreadingHTTPServer((self._host, port), Handler)
                break
            except OSError as error:
                last_error = error
        if self._server is None:
            raise RuntimeError(f"No local control port available: {last_error}")
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="isaac-lab-web-control",
            daemon=True,
        )
        self._thread.start()
        return self.url

    def drain(self, maximum=24):
        commands = []
        for _ in range(int(maximum)):
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                break
        return commands

    def update_status(self, status):
        """Publish an immutable main-thread snapshot to browser clients."""
        with self._status_lock:
            self._status = dict(status)

    def stop(self):
        server, thread = self._server, self._thread
        self._server = None
        self._thread = None
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
