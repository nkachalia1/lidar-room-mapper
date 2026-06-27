from __future__ import annotations

import json
import mimetypes
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable
from urllib.parse import urlparse

from lidar_room_mapper.runtime import MappingRuntime


class DashboardServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        runtime: MappingRuntime,
        static_dir: Path | None = None,
    ) -> None:
        self.runtime = runtime
        self.static_dir = static_dir or Path(__file__).with_name("static")
        super().__init__(server_address, DashboardRequestHandler)


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: DashboardServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        routes: dict[str, Callable[[], None]] = {
            "/api/state": self._state,
            "/api/reset": self._reset,
            "/api/latest.jpg": self._latest_image,
        }
        if path in routes:
            routes[path]()
            return
        if path == "/":
            self._static("index.html")
            return
        self._static(path.lstrip("/"))

    def log_message(self, format: str, *args: object) -> None:
        return None

    def _state(self) -> None:
        self._json(self.server.runtime.snapshot())

    def _reset(self) -> None:
        self.server.runtime.reset()
        self._json({"ok": True})

    def _latest_image(self) -> None:
        snapshot = self.server.runtime.snapshot()
        camera = snapshot.get("camera", {})
        path = camera.get("path") if isinstance(camera, dict) else None
        if not path:
            self.send_error(HTTPStatus.NOT_FOUND, "No camera frame captured yet.")
            return
        image_path = Path(str(path))
        if not image_path.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "Latest camera frame does not exist.")
            return
        self._send_bytes(image_path.read_bytes(), "image/jpeg")

    def _static(self, relative_path: str) -> None:
        safe_path = Path(relative_path)
        if safe_path.is_absolute() or ".." in safe_path.parts:
            self.send_error(HTTPStatus.BAD_REQUEST)
            return

        file_path = self.server.static_dir / safe_path
        if not file_path.exists() or not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        self._send_bytes(file_path.read_bytes(), content_type)

    def _json(self, payload: dict[str, object]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self._send_bytes(data, "application/json; charset=utf-8")

    def _send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)
