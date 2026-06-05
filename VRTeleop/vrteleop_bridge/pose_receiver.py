from __future__ import annotations

import json
import socket
from contextlib import AbstractContextManager

from .types import ControllerPose


class UdpPoseReceiver(AbstractContextManager["UdpPoseReceiver"]):
    def __init__(self, host: str, port: int, timeout_sec: float) -> None:
        self.host = host
        self.port = port
        self.timeout_sec = timeout_sec
        self._sock: socket.socket | None = None

    def __enter__(self) -> "UdpPoseReceiver":
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(self.timeout_sec)
        return self

    def __exit__(self, *exc: object) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def recv(self) -> ControllerPose | None:
        if self._sock is None:
            raise RuntimeError("UdpPoseReceiver is not open.")
        try:
            packet, _addr = self._sock.recvfrom(8192)
        except socket.timeout:
            return None
        data = json.loads(packet.decode("utf-8"))
        return ControllerPose(
            seq=int(data["seq"]),
            timestamp_usec=int(data["timestamp_usec"]),
            position=tuple(float(v) for v in data["position"]),
            orientation_xyzw=tuple(float(v) for v in data["orientation_xyzw"]),
            trigger=float(data.get("trigger", 0.0)),
            grip=float(data.get("grip", 0.0)),
            enabled=bool(data.get("enabled", False)),
        )
