"""
PathVizMonitor — drop-in replacement for MujocoMonitor that overlays
path planning data (waypoints, executed trails, pass targets) received
from agents via UDP into the native MuJoCo viewer.

Usage: replace MujocoMonitor with PathVizMonitor wherever the monitor
is instantiated in the server startup code.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import defaultdict
from collections.abc import Sequence
from typing import Any

import glfw
import mujoco
import numpy as np

from rcsssmj.games.soccer.sim.soccer_commands import DropBallCommand, KickOffCommand
from rcsssmj.games.soccer.sim.soccer_state_info import SoccerEnvironmentInformation, SoccerGameInformation
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.server.remote_monitor import RemoteMonitorState, SimMonitor
from rcsssmj.sim.state_info import SceneGraph, SimStateInformation

try:
    from OpenGL import GL
except ImportError:
    GL = None

# ── colour palette ─────────────────────────────────────────────────────────────
_TEAM_COLOURS: dict[str, tuple[float, float, float]] = {}
_PALETTE = [
    (0.11, 0.74, 0.53),  # teal
    (0.95, 0.37, 0.18),  # coral
    (0.20, 0.56, 0.90),  # blue
    (0.85, 0.65, 0.12),  # amber
]
_TARGET_COLOUR  = np.array([1.0, 0.9, 0.1, 0.9], dtype=np.float32)
_TRAIL_ALPHA    = 0.35
_PLAN_ALPHA     = 0.85
_WAYPOINT_RADIUS = 0.08
_CONNECTOR_WIDTH = 0.015
_TARGET_RADIUS   = 0.12


def _team_rgb(team: str) -> tuple[float, float, float]:
    if team not in _TEAM_COLOURS:
        _TEAM_COLOURS[team] = _PALETTE[len(_TEAM_COLOURS) % len(_PALETTE)]
    return _TEAM_COLOURS[team]


def _rgba(rgb: tuple[float, float, float], alpha: float) -> np.ndarray:
    return np.array([*rgb, alpha], dtype=np.float32)


def _identity3x3() -> np.ndarray:
    return np.eye(3, dtype=np.float64).flatten()


class PathVizMonitor(MujocoMonitor):
    """
    MujocoMonitor subclass that overlays path planning geoms.

    Agents push JSON payloads via UDP (default port 60002):
        {
            "player":       1,
            "team":         "TeamA",
            "plan":         [[x0,y0], [x1,y1], ...],
            "current_step": 3,
            "trail":        [[x, y]],
            "target":       [x, y] | null
        }
    All coordinates are world-space (x, y). Z is fixed to GEOM_Z so
    overlays float just above the pitch surface.
    """

    GEOM_Z        = 0.05
    UDP_PORT      = 60002
    MAX_GEOMS     = 2000
    TRAIL_HISTORY = 40

    def __init__(self, model: Any, render_interval: int, udp_port: int = UDP_PORT) -> None:
        super().__init__(model, render_interval)

        # Increase scene geom budget beyond the parent's default of 1000
        self.scene = mujoco.MjvScene(model, self.MAX_GEOMS)

        # Latest path payload per (team, player_num)
        self._paths: dict[tuple[str, int], dict] = {}
        self._paths_lock = threading.Lock()

        # Persistent trail accumulator — survives between UDP packets
        self._trails: dict[tuple[str, int], list] = defaultdict(list)

        # UDP listener
        self._udp_port = udp_port
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.bind(("", udp_port))
        self._udp_sock.settimeout(1.0)
        self._running = True
        self._listener = threading.Thread(
            target=self._udp_listen, daemon=True, name="PathVizUDP"
        )
        self._listener.start()
        print(f"[PathVizMonitor] listening for path data on UDP :{udp_port}")

    # ── UDP listener ───────────────────────────────────────────────────────────

    def _udp_listen(self) -> None:
        while self._running:
            try:
                raw, _ = self._udp_sock.recvfrom(65535)
                payload = json.loads(raw.decode())
                key = (payload["team"], payload["player"])

                # Accumulate trail positions
                if payload.get("trail"):
                    with self._paths_lock:
                        hist = self._trails[key]
                        hist.extend(payload["trail"])
                        if len(hist) > self.TRAIL_HISTORY:
                            self._trails[key] = hist[-self.TRAIL_HISTORY:]

                with self._paths_lock:
                    self._paths[key] = payload

            except TimeoutError:
                continue
            except (json.JSONDecodeError, KeyError):
                continue

    # ── Geom helpers ───────────────────────────────────────────────────────────

    def _add_sphere(
        self,
        scn: Any,
        pos: tuple[float, float, float],
        radius: float,
        colour: np.ndarray,
    ) -> bool:
        if scn.ngeom >= scn.maxgeom:
            return False
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0], dtype=np.float64),
            np.array(pos, dtype=np.float64),
            _identity3x3(),
            colour,
        )
        g.objtype  = mujoco.mjtObj.mjOBJ_UNKNOWN
        g.objid    = -1
        g.category = mujoco.mjtCatBit.mjCAT_DECOR
        scn.ngeom += 1
        return True

    def _add_connector(
        self,
        scn: Any,
        p1: tuple[float, float, float],
        p2: tuple[float, float, float],
        width: float,
        colour: np.ndarray,
    ) -> bool:
        if scn.ngeom >= scn.maxgeom:
            return False
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_connector(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            width,
            np.array(p1, dtype=np.float64),
            np.array(p2, dtype=np.float64),
        )
        g.rgba[:]  = colour
        g.objtype  = mujoco.mjtObj.mjOBJ_UNKNOWN
        g.objid    = -1
        g.category = mujoco.mjtCatBit.mjCAT_DECOR
        scn.ngeom += 1
        return True

    def _xy(self, pt: list[float]) -> tuple[float, float, float]:
        return (float(pt[0]), float(pt[1]), self.GEOM_Z)

    # ── Core render override ───────────────────────────────────────────────────

    def render(self, data: Any) -> None:
        if self._state == RemoteMonitorState.SHUTDOWN:
            return

        # FPS tracking
        current_time = time.time()
        fps = 1 / (current_time - self.last_render_time)
        self.last_render_time = current_time
        self.fps = (self.fps * 10 + fps) / 11

        # Step 1: populate self.scene from physics state
        mujoco.mjv_updateScene(
            self.model, data, self.scene_option, None,
            self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )

        # Step 2: inject path overlay geoms AFTER mjv_updateScene (which
        # resets ngeom), BEFORE mjr_render consumes the scene
        with self._paths_lock:
            snapshot       = dict(self._paths)
            trail_snapshot = {k: list(v) for k, v in self._trails.items()}

        for (team, player_num), payload in snapshot.items():
            rgb = _team_rgb(team)

            # Executed trail — muted capsule chain
            trail = trail_snapshot.get((team, player_num), [])
            if len(trail) >= 2:
                trail_colour = _rgba(rgb, _TRAIL_ALPHA)
                for i in range(len(trail) - 1):
                    self._add_connector(
                        self.scene,
                        self._xy(trail[i]),
                        self._xy(trail[i + 1]),
                        _CONNECTOR_WIDTH * 0.6,
                        trail_colour,
                    )

            # Planned path — bright spheres at waypoints + connecting lines
            plan = payload.get("plan", [])
            plan_colour = _rgba(rgb, _PLAN_ALPHA)
            if plan:
                for wp in plan:
                    self._add_sphere(self.scene, self._xy(wp), _WAYPOINT_RADIUS, plan_colour)
                for i in range(len(plan) - 1):
                    self._add_connector(
                        self.scene,
                        self._xy(plan[i]),
                        self._xy(plan[i + 1]),
                        _CONNECTOR_WIDTH,
                        plan_colour,
                    )

            # Pass/goal target — distinct yellow sphere
            target = payload.get("target")
            if target:
                self._add_sphere(self.scene, self._xy(target), _TARGET_RADIUS, _TARGET_COLOUR)

        # Step 3: render scene (now includes our geoms)
        self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(self.window)
        mujoco.mjr_render(self.viewport, self.scene, self.context)

        # Step 4: HUD, overlays, glfw (duplicated from parent to avoid
        # calling super().render() which would run mjv_updateScene again)
        if self.game_state is not None:
            w, h = self.viewport.width, self.viewport.height
            bar_h_start = 30
            bar_h = 40
            y0, y1 = h - bar_h - bar_h_start, h - bar_h_start
            play_mode_bar_y0, play_mode_bar_y1 = y0, y0 - bar_h

            max_team_str_len = 20
            max_score_str_len = 5
            max_time_str_len = 5
            left_team_txt  = (self.game_state.left_team  or 'Unknown')[:max_team_str_len]
            score_text     = f'{self.game_state.left_score}:{self.game_state.right_score}'[:max_score_str_len]
            right_team_txt = (self.game_state.right_team or 'Unknown')[:max_team_str_len]
            mins = int(self.game_state.play_time // 60)
            secs = int(self.game_state.play_time % 60)
            time_text      = f'{mins:02d}:{secs:02d}'[:max_time_str_len]
            play_mode_text = f'Playmode: {self.game_state.play_mode}'

            char_w    = 16
            box_w     = char_w * max_team_str_len
            mid_left  = 0.44 * w
            mid_right = 0.56 * w
            lx0, lx1  = mid_left  - box_w / 2, mid_left  + box_w / 2
            rx0, rx1  = mid_right - box_w / 2, mid_right + box_w / 2
            text_y    = int(h - bar_h_start - bar_h / 2 - 8)
            sx0, sx1  = lx1, rx0
            time_box_w = char_w * max_time_str_len
            tx0       = rx1 + 10
            tx1       = tx0 + time_box_w

            if GL is not None:
                GL.glPushAttrib(GL.GL_ALL_ATTRIB_BITS)
                GL.glDisable(GL.GL_DEPTH_TEST)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glDisable(GL.GL_CULL_FACE)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                GL.glDisable(GL.GL_TEXTURE_2D)
                GL.glMatrixMode(GL.GL_PROJECTION)
                GL.glPushMatrix()
                GL.glLoadIdentity()
                GL.glOrtho(0, w, 0, h, -1, 1)
                GL.glMatrixMode(GL.GL_MODELVIEW)
                GL.glPushMatrix()
                GL.glLoadIdentity()

                GL.glColor4f(0.2, 0.2, 0.2, 1.0)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(lx0-5, y0-5); GL.glVertex2f(tx1+5, y0-5)
                GL.glVertex2f(tx1+5, y1+5); GL.glVertex2f(lx0-5, y1+5)
                GL.glEnd()

                GL.glColor4f(0.2, 0.2, 0.2, 0.5)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(lx0-5, play_mode_bar_y0-5); GL.glVertex2f(tx1+5, play_mode_bar_y0-5)
                GL.glVertex2f(tx1+5, play_mode_bar_y1);   GL.glVertex2f(lx0-5, play_mode_bar_y1)
                GL.glEnd()

                GL.glColor4f(0.0, 0.0, 0.8, 0.8)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(lx0, y0); GL.glVertex2f(lx1, y0)
                GL.glVertex2f(lx1, y1); GL.glVertex2f(lx0, y1)
                GL.glEnd()

                GL.glColor4f(0.8, 0.0, 0.0, 0.8)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(rx0, y0); GL.glVertex2f(rx1, y0)
                GL.glVertex2f(rx1, y1); GL.glVertex2f(rx0, y1)
                GL.glEnd()

                GL.glPopMatrix()
                GL.glMatrixMode(GL.GL_PROJECTION)
                GL.glPopMatrix()
                GL.glPopAttrib()
                GL.glMatrixMode(GL.GL_MODELVIEW)

            for txt, x0 in [(left_team_txt, lx0), (right_team_txt, rx0)]:
                mujoco.mjr_text(
                    mujoco.mjtFont.mjFONT_NORMAL, txt, self.context,
                    (x0 + ((max_team_str_len - len(txt)) / 2 * char_w)) / w,
                    (text_y - 12) / h, 1.0, 1.0, 1.0,
                )

            sx_center = sx0 + (sx1 - sx0) / 2
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL, score_text, self.context,
                (sx_center - (len(score_text) / 2 * char_w) + 5) / w,
                (text_y - 12) / h, 1.0, 1.0, 1.0,
            )
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL, time_text, self.context,
                (tx0 + ((max_time_str_len - len(time_text)) / 2 * char_w)) / w,
                (text_y - 12) / h, 1.0, 1.0, 1.0,
            )
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL, play_mode_text, self.context,
                lx0 / w, ((text_y - 12) - bar_h) / h, 1.0, 1.0, 1.0,
            )

        if not self.hide_menu:
            overlays = self.create_overlays()
            for gridpos, [t1, t2] in overlays.items():
                mujoco.mjr_overlay(
                    mujoco.mjtFont.mjFONT_NORMAL, gridpos,
                    self.viewport, t1, t2, self.context,
                )

        glfw.swap_buffers(self.window)
        glfw.poll_events()
        self.frames += 1
        self.set_camera()

        if glfw.window_should_close(self.window):
            self.shutdown()

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self._running = False
        self._udp_sock.close()
        super().shutdown()