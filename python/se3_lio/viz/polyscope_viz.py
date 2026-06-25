"""Live Polyscope viewer for SE(3)-LIO (optional, real-time).

A lightweight interactive visualizer: the current scan (in the gravity-aligned
world frame) and the trajectory, with play/pause/step controls. Polyscope is an
optional dependency imported lazily — install with ``pip install se3-lio[viz]``.
Follows the Polyscope visualizers in KISS-ICP / GenZ-ICP.
"""

import datetime
import importlib

import numpy as np

START_BUTTON = " START\n[SPACE]"
PAUSE_BUTTON = " PAUSE\n[SPACE]"
NEXT_BUTTON = "NEXT\n [N]"
CENTER_BUTTON = "CENTER\n  [C]"
SCREENSHOT_BUTTON = "SHOT\n [S]"

BACKGROUND_COLOR = [0.07, 0.07, 0.10]
FRAME_COLOR = [0.85, 0.11, 0.38]
TRAJECTORY_COLOR = [0.12, 0.53, 0.90]
FRAME_PTS_SIZE = 0.05


def _lidar_to_world(pose, extrinsic, pts):
    """LiDAR-frame points (N, 3) into the world frame: world = pose @ extrinsic @ pt."""
    T = np.asarray(pose, float) @ np.asarray(extrinsic, float)
    return np.asarray(pts, float) @ T[:3, :3].T + T[:3, 3]


def _vec_align(a, b):
    """Rotation R (3x3) with R @ a == b, for unit vectors a and b."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-8:
        return np.eye(3)
    if c < -1.0 + 1e-8:
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def _gravity_align(grav):
    """Rotation mapping the world frame so gravity points along -z (z up)."""
    if grav is None:
        return np.eye(3)
    grav = np.asarray(grav, float)
    n = float(np.linalg.norm(grav))
    if n < 1e-9:
        return np.eye(3)
    return _vec_align(grav / n, np.array([0.0, 0.0, -1.0]))


class PolyscopeVisualizer:
    """Real-time viewer with the pipeline logger interface (``log_frame``)."""

    def __init__(self, extrinsic):
        try:
            self._ps = importlib.import_module("polyscope")
            self._gui = self._ps.imgui
        except ModuleNotFoundError:
            print('polyscope is not installed — run "pip install se3-lio[viz]"')
            raise SystemExit(1)

        self._extrinsic = np.asarray(extrinsic, float)
        self._R = None  # gravity-alignment rotation, fixed from the first frame
        self._trajectory = []
        self._frame_size = FRAME_PTS_SIZE
        self._block = True
        self._play = False

        self._ps.set_program_name("SE(3)-LIO Visualizer")
        self._ps.init()
        self._ps.set_up_dir("z_up")
        self._ps.set_ground_plane_mode("none")
        self._ps.set_background_color(BACKGROUND_COLOR)
        self._ps.set_verbosity(0)
        self._ps.set_build_default_gui_panels(False)
        self._ps.set_user_callback(self._gui_callback)

    def log_frame(self, stamp, pose, scan_pts, grav=None):
        pose = np.asarray(pose, float)
        if self._R is None:
            self._R = _gravity_align(grav)
        R = self._R
        world = _lidar_to_world(pose, self._extrinsic, scan_pts) @ R.T
        self._trajectory.append(R @ pose[:3, 3])
        self._draw(world)

        # Step/play loop (KISS-ICP style): block per frame until NEXT/PLAY.
        while self._block:
            self._ps.frame_tick()
            if self._play:
                break
        self._block = not self._block

    def hold(self):
        """Keep the window open after the run so the final state can be inspected."""
        self._play = False
        self._ps.show()

    def _draw(self, world):
        if world.shape[0] > 0:
            fc = self._ps.register_point_cloud(
                "current_frame", world, color=FRAME_COLOR, point_render_mode="quad"
            )
            fc.set_radius(self._frame_size, relative=False)
        traj = np.asarray(self._trajectory)
        if len(traj) >= 2:
            edges = np.column_stack([np.arange(len(traj) - 1), np.arange(1, len(traj))])
            tn = self._ps.register_curve_network(
                "trajectory", traj, edges, color=TRAJECTORY_COLOR
            )
            tn.set_radius(0.04, relative=False)

    def _gui_callback(self):
        name = PAUSE_BUTTON if self._play else START_BUTTON
        if self._gui.Button(name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_Space):
            self._play = not self._play
        self._gui.SameLine()
        if self._gui.Button(NEXT_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_N):
            self._block = not self._block
        self._gui.SameLine()
        if self._gui.Button(CENTER_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_C):
            self._ps.reset_camera_to_home_view()
        self._gui.SameLine()
        if self._gui.Button(SCREENSHOT_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_S):
            self._ps.screenshot(
                "se3lio_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".jpg"
            )
        changed, self._frame_size = self._gui.SliderFloat(
            "frame size", self._frame_size, v_min=0.01, v_max=0.4
        )
        if changed and self._ps.has_point_cloud("current_frame"):
            self._ps.get_point_cloud("current_frame").set_radius(self._frame_size, relative=False)
