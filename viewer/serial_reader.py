"""Serial communication with VL53L5CX sensor via ESP32."""

import json
import logging
import math
import threading
import time

import numpy as np
import serial

from . import config

logger = logging.getLogger("vl53l5cx_viewer.serial")


class SerialReader:
    """Background thread for reading sensor data over serial."""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None

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
        logger.info("Connecting to %s at %d baud...", self.port, self.baud)
        self.serial = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)
        self.serial.reset_input_buffer()
        logger.info("Serial connected")

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
            if self.serial:
                self.serial.cancel_read()
        except Exception:
            pass

    def get_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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

    def _reconnect(self) -> bool:
        try:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass

            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.serial.reset_input_buffer()
            logger.info("Serial reconnected")
            return True

        except (serial.SerialException, OSError) as e:
            logger.debug("Reconnection failed: %s", e)
            return False

    def _read_loop(self):
        logger.info("Serial reader thread started")

        while self._running:
            try:
                if not self.serial or not self.serial.is_open:
                    break

                try:
                    line = self.serial.readline()
                except Exception:
                    break

                if not line:
                    continue

                line_str = line.decode("utf-8", errors="ignore").strip()

                if line_str.startswith("{"):
                    try:
                        data = json.loads(line_str)

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

                    except json.JSONDecodeError:
                        pass

            except (serial.SerialException, OSError):
                if not self._running:
                    break

                logger.warning("Serial connection lost")
                with self._data_lock:
                    self._data_fps = 0.0

                while self._running:
                    if self._reconnect():
                        break
                    time.sleep(1)

        logger.info("Serial reader thread stopped")

