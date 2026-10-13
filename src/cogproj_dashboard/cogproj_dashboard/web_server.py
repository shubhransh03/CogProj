"""Tornado-based HTTP and WebSocket server for CogProj Operator Console."""

import asyncio
import json
import logging
import os
import threading
from typing import Any, Dict, List, Optional, Set

import tornado.web
import tornado.websocket

logger = logging.getLogger("cogproj_dashboard.web_server")


class DashboardWebSocketHandler(tornado.websocket.WebSocketHandler):
    """Bi-directional WebSocket handler for telemetry streaming and control."""

    clients: Set["DashboardWebSocketHandler"] = set()
    node_ref: Any = None

    def check_origin(self, origin: str) -> bool:
        """Allow connections from localhost and forwarded SSH ports."""
        return True

    def open(self) -> None:
        DashboardWebSocketHandler.clients.add(self)
        logger.info(f"Dashboard client connected: {self.request.remote_ip} (Total: {len(self.clients)})")
        if self.node_ref:
            # Send initial full state immediately
            self.send_json(self.node_ref.get_full_state())

    def on_close(self) -> None:
        DashboardWebSocketHandler.clients.discard(self)
        logger.info(f"Dashboard client disconnected. (Remaining: {len(self.clients)})")
        if len(self.clients) == 0 and self.node_ref:
            # Safety: no clients connected -> stop vehicle motion
            self.node_ref.supervisor.handle_disconnect()

    def on_message(self, message: str) -> None:
        if not self.node_ref:
            return

        try:
            data = json.loads(message)
            msg_type = data.get("type", "")

            if msg_type == "set_mode":
                mode = data.get("mode", "STOPPED")
                success, msg = self.node_ref.supervisor.set_mode(mode)
                self.send_json({"type": "mode_response", "success": success, "message": msg, "mode": self.node_ref.supervisor.mode.value})

            elif msg_type == "teleop":
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
                deadman = bool(data.get("deadman", False))
                success, msg = self.node_ref.supervisor.receive_teleop_command(linear, angular, deadman)

            elif msg_type == "nav_goal":
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                yaw = float(data.get("yaw", 0.0))
                self.node_ref.dispatch_nav_goal(x, y, yaw)

            elif msg_type == "cancel_nav":
                self.node_ref.cancel_nav_goal()

            elif msg_type == "estop":
                self.node_ref.supervisor.emergency_stop()
                self.send_json({"type": "mode_response", "success": True, "message": "Emergency Stop Engaged", "mode": "STOPPED"})

        except Exception as e:
            logger.error(f"Error handling WebSocket message: {e}", exc_info=True)

    def send_json(self, data: Dict[str, Any]) -> None:
        try:
            self.write_message(json.dumps(data))
        except tornado.websocket.WebSocketClosedError:
            pass

    @classmethod
    def broadcast_json(cls, data: Dict[str, Any]) -> None:
        if not cls.clients:
            return
        payload = json.dumps(data)
        for client in list(cls.clients):
            try:
                client.write_message(payload)
            except Exception:
                cls.clients.discard(client)


class ApiStatusHandler(tornado.web.RequestHandler):
    """Lightweight REST endpoint for status checks."""

    node_ref: Any = None

    def get(self) -> None:
        self.set_header("Content-Type", "application/json")
        if self.node_ref:
            self.write(json.dumps(self.node_ref.get_full_state()))
        else:
            self.write(json.dumps({"status": "starting"}))


class WebServerThread:
    """Manages the background asyncio event loop running Tornado."""

    def __init__(self, node_ref: Any, host: str = "127.0.0.1", port: int = 8080) -> None:
        self.node_ref = node_ref
        self.host = host
        self.port = int(port)
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._server: Optional[tornado.httpserver.HTTPServer] = None

        # Wire node_ref to handlers
        DashboardWebSocketHandler.node_ref = node_ref
        ApiStatusHandler.node_ref = node_ref

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_server, daemon=True, name="DashboardWebServer")
        self._thread.start()

    def _run_server(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        static_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
        if not os.path.exists(static_dir):
            # If installed, check share directory
            from ament_index_python.packages import get_package_share_directory
            try:
                pkg_dir = get_package_share_directory("cogproj_dashboard")
                static_dir = os.path.join(pkg_dir, "static")
            except Exception:
                pass

        app = tornado.web.Application(
            [
                (r"/api/status", ApiStatusHandler),
                (r"/ws", DashboardWebSocketHandler),
                (r"/(.*)", tornado.web.StaticFileHandler, {"path": static_dir, "default_filename": "index.html"}),
            ]
        )

        self._server = tornado.httpserver.HTTPServer(app)
        self._server.listen(self.port, address=self.host)
        logger.info(f"CogProj Operator Console serving on http://{self.host}:{self.port}")
        self._loop.run_forever()

    def stop(self) -> None:
        if self._server:
            self._server.stop()
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
