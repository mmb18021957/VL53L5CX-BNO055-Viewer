import json
import logging
import threading
import time
import socket
import numpy as np

from . import config

logger = logging.getLogger("vl53l5cx_viewer.udp")


class UDPReader:
    """Background thread for reading sensor data over WiFi UDP."""

    def __init__(self, port: int = 9999):
        self.port = port
        self.running = False

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

        self._thread = None

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.1)

    @property
    def data_fps(self):
        with self._data_lock:
            return self._data_fps

    @property
    def imu_connected(self):
        with self._data_lock:
            return self._imu_connected

    def connect(self):
        """UDP requires no connection setup."""
        pass

    def start(self):
        if self._thread is not None:
            return
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def get_data(self):
        with self._data_lock:
            return self.distances.copy(), self.status.copy(), self.quaternion.copy()

    def _read_loop(self):
        logger.info("UDP reader thread started on port %d", self.port)
        while self.running:
            try:
                data_raw, _ = self.sock.recvfrom(2048)
                data_str = data_raw.decode("utf-8", errors="ignore").strip()

                if not data_str.startswith("{"):
                    continue

                data = json.loads(data_str)

                if "distances" not in data or "status" not in data:
                    continue

                distances = data["distances"]
                status = data["status"]

                if len(distances) != config.NUM_ZONES:
                    continue

                with self._data_lock:
                    self.distances = np.array(distances, dtype=np.float32)
                    self.status = np.array(status, dtype=np.uint8)

                    if "quat" in data:
                        q = np.array(data["quat"], dtype=np.float32)
                        n = np.linalg.norm(q)
                        if n > 0:
                            q /= n
                        self.quaternion = q
                        self._imu_connected = True

                # FPS tracking
                self._frame_count += 1
                now = time.time()
                if now - self._last_fps_time >= 1.0:
                    with self._data_lock:
                        self._data_fps = self._frame_count / (now - self._last_fps_time)
                    self._frame_count = 0
                    self._last_fps_time = now

            except socket.timeout:
                pass
            except Exception as e:
                logger.warning("UDP error: %s", e)
