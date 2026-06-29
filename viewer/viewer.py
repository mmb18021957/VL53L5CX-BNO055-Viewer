#!/usr/bin/env python3
"""VL53L5CX Point Cloud Viewer - neue Standard-Version mit stabilem Mapping."""

import argparse
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import viser
from scipy.spatial.transform import Rotation

def hue_to_rgb(h):
    h = float(h) / 255.0
    r = int(np.sin(2*np.pi*h) * 127 + 128)
    g = int(np.sin(2*np.pi*(h + 1/3)) * 127 + 128)
    b = int(np.sin(2*np.pi*(h + 2/3)) * 127 + 128)
    return np.array([r, g, b], dtype=np.uint8)


from . import config
from .filters import TemporalFilter, fit_plane, fit_plane_ransac
from .geometry import (
    CoordinateMethod,
    compute_zone_angles,
    correct_imu_to_tof_frame,
    distances_to_points,
    get_colors,
)
from .logging_config import setup_logging
from .scene import create_grid, create_scene_hierarchy, update_zone_rays


logger = logging.getLogger("vl53l5cx_viewer.main")


# ---------------------------------------------------------------------------
# Mapping State
# ---------------------------------------------------------------------------

@dataclass
class MappingState:
    """State for mapping mode point accumulation (world coordinates)."""

    accumulated_points: list[np.ndarray] = field(default_factory=list)
    accumulated_colors: list[np.ndarray] = field(default_factory=list)
    _clear_requested: bool = False

    def request_clear(self) -> None:
        self._clear_requested = True

    def process_clear_if_requested(self) -> bool:
        if self._clear_requested:
            self._clear_requested = False
            self.accumulated_points.clear()
            self.accumulated_colors.clear()
            return True
        return False

    def add(self, points: np.ndarray, colors: np.ndarray) -> None:
        self.accumulated_points.append(points)
        self.accumulated_colors.append(colors)

    def get_display_data(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.accumulated_points:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)
        if len(self.accumulated_points) == 1:
            return self.accumulated_points[0], self.accumulated_colors[0]
        return np.vstack(self.accumulated_points), np.vstack(self.accumulated_colors)

    def total_points(self) -> int:
        return sum(len(p) for p in self.accumulated_points)

    def downsample(self, voxel_size: float, max_points: int) -> None:
        if not self.accumulated_points:
            return
        all_points = np.vstack(self.accumulated_points)
        all_colors = np.vstack(self.accumulated_colors)

        voxel_indices = np.ascontiguousarray(
            np.floor(all_points / voxel_size).astype(np.int64)
        )
        keys = voxel_indices.view(
            dtype=[("x", np.int64), ("y", np.int64), ("z", np.int64)]
        ).ravel()
        _, unique_idx = np.unique(keys, return_index=True)

        all_points = all_points[unique_idx]
        all_colors = all_colors[unique_idx]

        if len(all_points) > max_points:
            all_points = all_points[-max_points:]
            all_colors = all_colors[-max_points:]

        self.accumulated_points = [all_points]
        self.accumulated_colors = [all_colors]


# ---------------------------------------------------------------------------
# Viewer Class
# ---------------------------------------------------------------------------

class VL53L5CXViewer:
    """Real-time point cloud viewer for VL53L5CX ToF sensor."""

    def __init__(self, reader):
        self.serial_reader = reader

        self.zone_angles = compute_zone_angles()
        self.temporal_filter = TemporalFilter()

        self.imu_board_center = (
            np.array(config.IMU_BOARD.world_position)
            - np.array(config.IMU_BOARD.sensor_offset)
        )
        self.tof_board_center = (
            np.array(config.TOF_BOARD.world_position)
            - np.array(config.TOF_BOARD.sensor_offset)
        )
        self.imu_to_tof_offset = (
            np.array(config.TOF_BOARD.world_position)
            - np.array(config.IMU_BOARD.world_position)
        )

        self._ray_update_counter = 0
        self._rays_dirty = False
        self._last_ray_update = 0.0
        self._ray_update_interval = 0.01
        self.target_resolution = 8

    # -----------------------------------------------------------------------
    # Scene Setup
    # -----------------------------------------------------------------------

    def _setup_scene(self, server: viser.ViserServer) -> None:
        server.scene.add_frame("/origin", axes_length=0.002, axes_radius=0.0001)
        create_grid(server)
        assets_dir = Path(__file__).parent.parent / "assets"
        self.scene = create_scene_hierarchy(server, assets_dir, self.zone_angles)

    # -----------------------------------------------------------------------
    # GUI Setup
    # -----------------------------------------------------------------------

    def _setup_gui(self, server: viser.ViserServer, mapping_state: MappingState) -> None:
        self.imu_yaw = server.gui.add_slider(
            "Yaw (deg)", min=-180, max=180, step=1, initial_value=130.0
        )

        # Sensor Info
        with server.gui.add_folder("Sensor Info"):
            self.distance_text = server.gui.add_text("Status", initial_value="Waiting...")
            self.freq_text = server.gui.add_text("Frequency (Hz)", initial_value="0")
            self.imu_status_text = server.gui.add_text("IMU", initial_value="Not detected")

        # Settings
        with server.gui.add_folder("Settings"):
            self.point_size_slider = server.gui.add_slider(
                "Point Size", min=0.001, max=0.020, step=0.001, initial_value=0.005
            )
            self.show_rays_checkbox = server.gui.add_checkbox(
                "Show Zone Rays", initial_value=True
            )
            self.clip_rays_checkbox = server.gui.add_checkbox(
                "Clip to Measurement", initial_value=True
            )

            self.resolution_dropdown = server.gui.add_dropdown(
                "Resolution (zones)", options=["8", "4", "2", "1"], initial_value="8"
            )

            @self.resolution_dropdown.on_update
            def _on_res_change(event: viser.GuiEvent) -> None:
                self.target_resolution = int(self.resolution_dropdown.value)

            @self.show_rays_checkbox.on_update
            def _on_show_rays(event: viser.GuiEvent) -> None:
                self.clip_rays_checkbox.disabled = not self.show_rays_checkbox.value
                self._rays_dirty = True

            @self.clip_rays_checkbox.on_update
            def _on_clip_rays(event: viser.GuiEvent) -> None:
                self._rays_dirty = True

            self.coord_method_dropdown = server.gui.add_dropdown(
                "Coordinate Method",
                options=[m.value for m in CoordinateMethod],
                initial_value=CoordinateMethod.UNIFORM.value,
            )

            @self.coord_method_dropdown.on_update
            def _on_coord_method(event: viser.GuiEvent) -> None:
                self._rays_dirty = True

            self.imu_rotation_checkbox = server.gui.add_checkbox(
                "Apply IMU Rotation", initial_value=True
            )

            self.filter_checkbox = server.gui.add_checkbox(
                "Enable Filtering", initial_value=False
            )
            self.filter_strength_slider = server.gui.add_slider(
                "Filter Strength", min=0.0, max=1.0, step=0.05,
                initial_value=0.5, disabled=True
            )

            @self.filter_checkbox.on_update
            def _on_filter_toggle(event: viser.GuiEvent) -> None:
                self.filter_strength_slider.disabled = not self.filter_checkbox.value
                if not self.filter_checkbox.value:
                    self.temporal_filter.reset()

            self.fit_plane_checkbox = server.gui.add_checkbox(
                "Fit Plane", initial_value=False
            )
            self.plane_method_dropdown = server.gui.add_dropdown(
                "Method", options=["Least Squares", "RANSAC"], initial_value="Least Squares"
            )
            self.ransac_threshold_slider = server.gui.add_slider(
                "RANSAC Threshold (mm)", min=1, max=50, step=1, initial_value=10,
                visible=False
            )
            self.plane_error_text = server.gui.add_text(
                "Plane RMSE (mm)", initial_value="--"
            )

            @self.fit_plane_checkbox.on_update
            def _on_fit_plane_toggle(event: viser.GuiEvent) -> None:
                self.plane_method_dropdown.disabled = not self.fit_plane_checkbox.value
                self.ransac_threshold_slider.visible = (
                    self.fit_plane_checkbox.value
                    and self.plane_method_dropdown.value == "RANSAC"
                )
                if not self.fit_plane_checkbox.value:
                    self.plane_error_text.value = "--"

            @self.plane_method_dropdown.on_update
            def _on_plane_method_change(event: viser.GuiEvent) -> None:
                self.ransac_threshold_slider.visible = (
                    self.plane_method_dropdown.value == "RANSAC"
                )

        # Mapping
        with server.gui.add_folder("Mapping"):
            self.mapping_checkbox = server.gui.add_checkbox(
                "Mapping Mode", initial_value=False
            )

            # Neuer Color-Slider (0–255)
            self.map_color_slider = server.gui.add_slider(
                "Map Color (Hue)", min=0, max=255, step=1, initial_value=120            
            )
            self.voxel_size_slider = server.gui.add_slider(
                "Voxel Size (mm)", min=5, max=50, step=5, initial_value=10
            )
            self.max_s_slider = server.gui.add_slider(
                "Max Points (k)", min=10, max=500, step=10, initial_value=100
            )
            self.point_count_text = server.gui.add_text("Points", initial_value="0")
            clear_button = server.gui.add_button("Clear Map")

            @clear_button.on_click
            def _on_clear_click(event: viser.GuiEvent) -> None:
                mapping_state.request_clear()

            @self.mapping_checkbox.on_update
            def _on_mapping_toggle(event):
                if self.mapping_checkbox.value:
                    self.current_map_color = hue_to_rgb(self.map_color_slider.value)
                    server.scene.remove_by_name("/breadboard/tof/sensor/points")

        # IMU Board Position
        with server.gui.add_folder("IMU Board Position"):
            self.imu_x = server.gui.add_slider(
                "X (m)", min=-2.0, max=2.0, step=0.01,
                initial_value=float(config.IMU_BOARD.world_position[0])
            )
            self.imu_y = server.gui.add_slider(
                "Y (m)", min=-2.0, max=2.0, step=0.01,
                initial_value=float(config.IMU_BOARD.world_position[1])
            )
            self.imu_z = server.gui.add_slider(
                "Z (m)", min=-4.0, max=4.0, step=0.01,
                initial_value=float(config.IMU_BOARD.world_position[2])
            )

            @self.imu_x.on_update
            @self.imu_y.on_update
            @self.imu_z.on_update
            def _on_imu_pos_change(event: viser.GuiEvent) -> None:
                config.IMU_BOARD.world_position = (
                    self.imu_x.value,
                    self.imu_y.value,
                    self.imu_z.value,
                )

    # -----------------------------------------------------------------------
    # Scene Transforms
    # -----------------------------------------------------------------------

    def _update_scene_transforms(
        self, corrected_quat: np.ndarray, imu_connected: bool, apply_rotation: bool
    ) -> tuple[np.ndarray, Rotation] | None:
        imu_sensor_pos = np.array(config.IMU_BOARD.world_position)

        if apply_rotation and imu_connected:
            imu_rot = Rotation.from_quat(
                [corrected_quat[1], corrected_quat[2], corrected_quat[3], corrected_quat[0]]
            )

            user_yaw = Rotation.from_euler("z", self.imu_yaw.value, degrees=True)
            imu_rot = user_yaw * imu_rot

            imu_quat = imu_rot.as_quat()
            self.scene.imu_board.wxyz = (imu_quat[3], imu_quat[0], imu_quat[1], imu_quat[2])

            imu_board_offset = -np.array(config.IMU_BOARD.sensor_offset)
            rotated_imu_board_offset = imu_rot.apply(imu_board_offset)
            self.scene.imu_board.position = tuple(imu_sensor_pos + rotated_imu_board_offset)

            tof_sensor_pos = imu_sensor_pos + imu_rot.apply(self.imu_to_tof_offset)

            tof_quat = imu_rot.as_quat()
            self.scene.tof_board.wxyz = (tof_quat[3], tof_quat[0], tof_quat[1], tof_quat[2])

            tof_board_offset = -np.array(config.TOF_BOARD.sensor_offset)
            rotated_tof_board_offset = imu_rot.apply(tof_board_offset)
            self.scene.tof_board.position = tuple(tof_sensor_pos + rotated_tof_board_offset)

            return tof_sensor_pos, imu_rot

        else:
            self.scene.imu_board.wxyz = (1.0, 0.0, 0.0, 0.0)
            self.scene.imu_board.position = tuple(self.imu_board_center)
            self.scene.tof_board.wxyz = (1.0, 0.0, 0.0, 0.0)
            self.scene.tof_board.position = tuple(self.tof_board_center)
            return None

    # -----------------------------------------------------------------------
    # Frame Processing (mit stabilem Mapping)
    # -----------------------------------------------------------------------

    def _process_frame(
        self, server: viser.ViserServer, mapping_state: MappingState, plane_handle
    ):
        distances, status, quaternion = self.serial_reader.get_data()

        if self.filter_checkbox.value:
            distances = self.temporal_filter.apply(
                distances, self.filter_strength_slider.value
            )

        imu_connected = self.serial_reader.imu_connected
        self.imu_status_text.value = "Connected" if imu_connected else "Not detected"

        corrected_quat = correct_imu_to_tof_frame(quaternion) if imu_connected else quaternion

        # Clear Map
        if mapping_state.process_clear_if_requested():
            self.point_count_text.value = "0"
            server.scene.remove_by_name("/map/points")

        if not np.any(distances):
            self.distance_text.value = "No valid data"
            return plane_handle

        coord_method = next(
            m for m in CoordinateMethod if m.value == self.coord_method_dropdown.value
        )

        points_local = distances_to_points(distances, self.zone_angles, coord_method)
        colors = get_colors(distances, status)
        valid_mask = np.isin(status, [5, 6, 9]) & (distances >= config.MIN_RANGE_MM)

        distances_full = distances.copy()

        # Downsampling (Resolution)
        if self.target_resolution < 8:
            factor = 8 // self.target_resolution

            points_local = points_local.reshape(8, 8, 3)
            colors = colors.reshape(8, 8, 3)
            valid_mask = valid_mask.reshape(8, 8)
            distances = distances.reshape(8, 8)

            points_local = points_local.reshape(
                self.target_resolution, factor,
                self.target_resolution, factor, 3
            ).mean(axis=(1, 3))

            colors = colors.reshape(
                self.target_resolution, factor,
                self.target_resolution, factor, 3
            ).mean(axis=(1, 3)).astype(np.uint8)

            distances = distances.reshape(
                self.target_resolution, factor,
                self.target_resolution, factor
            ).mean(axis=(1, 3))

            valid_mask = valid_mask.reshape(
                self.target_resolution, factor,
                self.target_resolution, factor
            ).any(axis=(1, 3))

            points_local = points_local.reshape(-1, 3)
            colors = colors.reshape(-1, 3)
            distances = distances.reshape(-1)
            valid_mask = valid_mask.reshape(-1)

        transform_result = self._update_scene_transforms(
            corrected_quat, imu_connected, self.imu_rotation_checkbox.value
        )

        if np.any(valid_mask):
            valid_local = points_local[valid_mask].astype(np.float32)
            valid_colors = colors[valid_mask]

            # Mapping: Punkte sammeln in Weltkoordinaten
            if self.mapping_checkbox.value:
                if transform_result is not None:
                    tof_sensor_pos, imu_rot = transform_result
                    sensor_yaw = Rotation.from_euler(
                        "z", config.TOF_BOARD.sensor_yaw_deg, degrees=True
                    )
                    world_rot = imu_rot * sensor_yaw
                    valid_world = world_rot.apply(valid_local) + tof_sensor_pos
                else:
                    sensor_yaw = Rotation.from_euler(
                        "z", config.TOF_BOARD.sensor_yaw_deg, degrees=True
                    )
                    valid_world = (
                        sensor_yaw.apply(valid_local)
                        + np.array(config.TOF_BOARD.world_position)
                    )

                # Farbe für diesen Layer aus Slider holen
                layer_colors = np.tile(self.current_map_color, (len(valid_world), 1))

                # Punkte + Layer-Farbe speichern
                mapping_state.add(valid_world, layer_colors)
                                
                

                if (
                    mapping_state.total_points() > config.DOWNSAMPLE_POINT_THRESHOLD
                    or len(mapping_state.accumulated_points)
                    > config.DOWNSAMPLE_BUFFER_THRESHOLD
                ):
                    voxel_size_m = self.voxel_size_slider.value / 1000.0
                    max_pts = self.max_s_slider.value * 1000
                    mapping_state.downsample(voxel_size_m, max_pts)

            # Map IMMER anzeigen
            display_points, display_colors = mapping_state.get_display_data()
            self.point_count_text.value = f"{len(display_points):,}"

            if len(display_points) > 0:
                server.scene.add_point_cloud(
                    "/map/points",
                    points=display_points,
                    colors=display_colors,
                    point_size=self.point_size_slider.value,
                    point_shape="circle",
                )

            # Live-Punkte nur wenn Mapping AUS
            if not self.mapping_checkbox.value:
                server.scene.add_point_cloud(
                    "/breadboard/tof/sensor/points",
                    points=valid_local,
                    colors=valid_colors,
                    point_size=self.point_size_slider.value,
                    point_shape="circle",
                )

            # Plane fitting
            if self.fit_plane_checkbox.value and len(valid_local) >= 3:
                if self.plane_method_dropdown.value == "RANSAC":
                    threshold_m = self.ransac_threshold_slider.value / 1000.0
                    plane_fit = fit_plane_ransac(valid_local, threshold=threshold_m)
                else:
                    plane_fit = fit_plane(valid_local)

                if plane_fit is not None:
                    pos, wxyz, size, rmse_mm = plane_fit
                    self.plane_error_text.value = f"{rmse_mm:.2f}"
                    plane_handle = server.scene.add_box(
                        "/breadboard/tof/sensor/plane",
                        dimensions=(size, size, 0.0001),
                        position=pos,
                        wxyz=wxyz,
                        color=(255, 255, 0),
                        opacity=0.5,
                    )

            valid_distances = distances[valid_mask]
            self.distance_text.value = (
                f"Range: {valid_distances.min():.0f}-{valid_distances.max():.0f}mm"
            )
        else:
            self.distance_text.value = "No valid data"

        if not self.fit_plane_checkbox.value and plane_handle is not None:
            plane_handle.remove()
            plane_handle = None

        self.freq_text.value = f"{self.serial_reader.data_fps:.1f}"

        # Rays (einfach, ohne Doppel-Logik)
        self._ray_update_counter += 1
        if self._ray_update_counter % 5 == 0 or self._rays_dirty:
            coord_method = next(
                m for m in CoordinateMethod if m.value == self.coord_method_dropdown.value
            )
            self.scene.zone_rays = update_zone_rays(
                server,
                self.zone_angles,
                coord_method,
                visible=self.show_rays_checkbox.value,
                distances=distances_full if self.clip_rays_checkbox.value else None,
            )
            self._rays_dirty = False

        return plane_handle

    # -----------------------------------------------------------------------
    # Run Loop
    # -----------------------------------------------------------------------

    def run(self, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.serial_reader.connect()
        self.serial_reader.start()

        server = viser.ViserServer(host=host, port=port)
        logger.info("Viser server started at http://localhost:%d", port)

        @server.on_client_connect
        def on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, -0.50, 0.50)
            client.camera.look_at = (0.0, 0.0, 0.0)
            client.camera.up = (0.0, 0.0, 1.0)
            client.camera.near = 0.001
            client.camera.fov = 0.35

        mapping_state = MappingState()
        self._setup_scene(server)
        self._setup_gui(server, mapping_state)

        plane_handle = None

        try:
            while True:
                frame_start = time.time()
                plane_handle = self._process_frame(server, mapping_state, plane_handle)
                elapsed = time.time() - frame_start
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.serial_reader.stop()
            if hasattr(self.serial_reader, "thread"):
                try:
                    self.serial_reader.thread.join()
                except RuntimeError:
                    pass
            if hasattr(self.serial_reader, "serial"):
                try:
                    self.serial_reader.serial.close()
                except Exception:
                    pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="VL53L5CX Point Cloud Viewer")
    parser.add_argument(
        "--port",
        "-p",
        default="/dev/cu.usbserial-0001",
        help="Serial port (default: /dev/cu.usbserial-0001)",
    )
    parser.add_argument(
        "--use-udp",
        action="store_true",
        help="Use UDP instead of serial",
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--host", default="0.0.0.0",
        help="Viser server host (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--viser-port", type=int, default=8080,
        help="Viser server port (default: 8080)",
    )
    parser.add_argument(
        "--debug", "-d", action="store_true",
        help="Enable debug logging",
    )
    args = parser.parse_args()

    setup_logging(level=logging.DEBUG if args.debug else logging.INFO)

    if args.use_udp:
        from .udp_reader import UDPReader
        reader = UDPReader(port=9999)
    else:
        from .serial_reader import SerialReader
        reader = SerialReader(args.port, args.baud)

    viewer = VL53L5CXViewer(reader)
    viewer.run(host=args.host, port=args.viser_port)


if __name__ == "__main__":
    main()


