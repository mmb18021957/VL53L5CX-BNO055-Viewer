"""UDP communication with VL53L5CX sensor via ESP32."""

import json
import logging
import math
import socket
import threading
import time

import numpy as np

from . import config

logger = logging.getLogger("vl53l5cx_viewer.udp")


class DummySerial:
    """Compatibility wrapper so viewer.py can call serial.close() and serial.cancel_read()."""
    def close(self):
        pass

    def cancel_read(self):
        pass


class UDPReader:
    """Background thread for reading sensor data over UDP."""

    def __init__(self, port: int = 9999):
        self.port = port

        # UDP socket
        self.sock: socket.socket | None = None

        # Serial compatibility (viewer.py expects this!)
        self.serial = DummySerial()

        # Thread control flags
        self.running = False
        self._running = False

        # Data storage
        self.distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._imu_connected = False
        self._data_lock = threading.Lock()

        # FPS tracking
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._data_fps = 0.0

        # Thread handle (viewer.py expects .thread)
        self._thread: threading.Thread | None = None
        self.thread: threading.Thread | None = None

        self._version_checked = False

    @property
    def data_fps(self) -> float:
        with self._data_lock:
            return self._data_fps

    @property
    def imu_connected(self) -> bool:
        with self._data_lock:
            return self._imu_connected

    def connect(self):
        """Open UDP socket."""
        logger.info("Opening UDP port %d...", self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        logger.info("UDP socket ready")

    def start(self):
        """Start the reader thread."""
        if self._thread is not None:
            return

        self.running = True
        self._running = True

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # viewer.py compatibility
        self.thread = self._thread

    def stop(self):
        """Stop the reader thread."""
        self.running = False
        self._running = False

        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def close(self):
        """Compatibility method for SerialReader API."""
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

    def get_data(self):
        with self._data_lock:
            return (
                self.distances.copy(),
                self.status.copy(),
                self.quaternion.copy(),
            )

    def _validate_distances(self, distances: list) -> bool:
        for d in distances:
            if not isinstance(d, (int, float)):
                return False
            if math.isnan(d) or math.isinf(d):
                return False
        return True

    def _validate_quaternion(self, quat: list) -> bool:
        if len(quat) != 4:
            return False
        for q in quat:
            if not isinstance(q, (int, float)):
                return False
            if math.isnan(q) or math.isinf(q):
                return False
        return True

    def _read_loop(self):
        logger.info("UDP reader thread started")

        while self._running:
            try:
                if not self.sock:
                    break

                try:
                    data_raw, _ = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except Exception:
                    break

                try:
                    data = json.loads(data_raw.decode("utf-8", errors="ignore"))
                except json.JSONDecodeError:
                    continue

                if "distances" in data and "status" in data:
                    distances = data["distances"]
                    status = data["status"]

                    if len(distances) != config.NUM_ZONES or len(status) != config.NUM_ZONES:
                        logger.warning(
                            "Invalid array lengths: distances=%d, status=%d (expected %d)",
                            len(distances), len(status), config.NUM_ZONES
                        )
                        continue

                    if not self._validate_distances(distances):
                        logger.warning("Invalid distance values detected (NaN/Inf)")
                        continue

                    if "quat" in data and not self._validate_quaternion(data["quat"]):
                        logger.warning("Invalid quaternion values detected (NaN/Inf)")
                        data.pop("quat")

                    if not self._version_checked:
                        self._version_checked = True
                        fw = data.get("v")
                        if fw is None:
                            logger.warning("No version in data. Firmware may be outdated.")
                        elif fw != config.VERSION:
                            logger.warning(
                                "Version mismatch: firmware=%s, viewer=%s",
                                fw, config.VERSION
                            )

                    with self._data_lock:
                        self.distances = np.array(distances, dtype=np.float32)
                        self.status = np.array(status, dtype=np.uint8)
                        if "quat" in data:
                            self.quaternion = np.array(data["quat"], dtype=np.float32)
                            self._imu_connected = True

                    # FPS tracking
                    self._frame_count += 1
                    now = time.time()
                    elapsed = now - self._last_fps_time
                    if elapsed >= 1.0:
                        with self._data_lock:
                            self._data_fps = self._frame_count / elapsed
                        self._frame_count = 0
                        self._last_fps_time = now

            except Exception:
                break

        logger.info("UDP reader thread stopped")

