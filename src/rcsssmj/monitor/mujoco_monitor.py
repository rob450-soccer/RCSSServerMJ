import time
from collections.abc import Sequence
from itertools import cycle
from typing import Any

import glfw
import mujoco

from rcsssmj.monitor.state import SceneGraph, SimStateInformation, SoccerEnvironmentInformation, SoccerGameInformation

try:
    from OpenGL import GL
except ImportError:
    GL = None

from rcsssmj.monitor.commands import DropBallCommand, KickOffCommand
from rcsssmj.monitor.sim_monitor import SimMonitor, SimMonitorState


class MujocoMonitor(SimMonitor):
    """Internal monitor component."""

    def __init__(self, model: Any, render_interval: int) -> None:
        """Construct a new internal monitor viewer.

        Parameter
        ---------
        model: MjModel
            The simulation model.

        render_interval: int
            The interval (in cycles) in which to render the state of the simulation. Set to 1 to render each cycle.
        """

        super().__init__(render_interval)

        self.model = model
        self.fps: float = 0.0

        self.game_state: SoccerGameInformation | None = None

        self.button_left: bool = False
        self.button_right: bool = False
        self.button_middle: bool = False
        self.last_x: int = 0
        self.last_y: int = 0
        self.frames: int = 0
        self.last_render_time: float = time.time()

        self.hide_menu: bool = True
        self.font_scale: int = 200

        glfw.init()
        glfw.window_hint(glfw.SCALE_TO_MONITOR, glfw.TRUE)

        primary_monitor = glfw.get_primary_monitor()
        video_mode = glfw.get_video_mode(primary_monitor)
        window_width, window_height = video_mode.size
        self.window = glfw.create_window(width=window_width, height=window_height, title='MuJoCo', monitor=None, share=None)
        glfw.make_context_current(self.window)

        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_scroll_callback(self.window, self.scroll)

        self.scene = mujoco.MjvScene(model, 1000)
        self.scene_option = mujoco.MjvOption()

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.all_camera_modes = ('static', 'follow')
        self.camera_mode_iter = cycle(self.all_camera_modes)
        self.camera_mode = next(self.camera_mode_iter)
        self.camera_mode_target = self.camera_mode
        self.set_camera()

        framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(self.window)
        self.viewport = mujoco.MjrRect(0, 0, framebuffer_width, framebuffer_height)
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale(self.font_scale))

        # Setting the swap interval to 0 forces the call to glfw.swap_buffers to not block even if the window is currently not visible (minimized or hidden by other windows).
        # This setting counteracts some energy saving measures performed by some OS, but is required to prevent rendering from blocking the execution of the main simulation loop.
        glfw.swap_interval(0)

    def shutdown(self, *, wait: bool = False) -> None:
        super().shutdown(wait=wait)

        glfw.set_window_should_close(self.window, True)
        glfw.destroy_window(self.window)
        glfw.poll_events()  # make sure there are no pending events, otherwise the window will just freeze and not close properly
        glfw.terminate()

    def update(self, state_info: Sequence[SimStateInformation], frame_id: int) -> None:
        scene_graph = None

        # process state information
        for info in state_info:
            if isinstance(info, SoccerEnvironmentInformation):
                # process environment information
                pass

            elif isinstance(info, SoccerGameInformation):
                # buffer soccer game state information
                self.game_state = info

            elif isinstance(info, SceneGraph):
                # buffer scene graph info
                scene_graph = info

        # check for valid scene graph information
        if scene_graph is not None:
            # check for model change
            if self.model is not scene_graph.mj_model:
                self.model = scene_graph.mj_model

                # recreate scene and context
                self.scene = mujoco.MjvScene(self.model, 1000)
                self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale(self.font_scale))

            # render
            if self.update_interval <= 1 or frame_id % self.update_interval == 0:
                self.render(scene_graph.mj_data)

    def mouse_button(self, window: Any, button: Any, act: Any, mods: Any) -> None:
        """Update mouse button state."""

        self.button_left = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self.button_right = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        self.button_middle = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS

        self.last_x, self.last_y = glfw.get_cursor_pos(self.window)

    def mouse_move(self, window: Any, x_pos: int, y_pos: int) -> None:
        """Handle mouse movement."""

        if not self.button_left and not self.button_right and not self.button_middle:
            return

        dx = x_pos - self.last_x
        dy = y_pos - self.last_y
        self.last_x, self.last_y = x_pos, y_pos

        width, height = glfw.get_window_size(self.window)

        mod_shift = glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS

        if self.button_right:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.model, action, dx / width, dy / height, self.scene, self.camera)

    def keyboard(self, window: Any, key: int, scancode: Any, act: int, mods: Any) -> None:
        """Handle keyboard inputs."""

        if act != glfw.RELEASE:
            return

        if key == glfw.KEY_H:
            self.hide_menu = not self.hide_menu
        elif key == glfw.KEY_TAB:
            self.camera_mode_target = next(self.camera_mode_iter)
        elif key == glfw.KEY_K:
            self._command_queue.put(KickOffCommand(0))
        elif key == glfw.KEY_J:
            self._command_queue.put(KickOffCommand(1))
        elif key == glfw.KEY_B:
            self._command_queue.put(DropBallCommand())

    def scroll(self, window: Any, x_offset: Any, y_offset: Any) -> None:
        """Handle scroll input."""

        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, 0.05 * y_offset, self.scene, self.camera)

    def render(self, data: Any) -> None:
        """Render function."""

        if self._state == SimMonitorState.DISCONNECTED:
            # prevent rendering after shutdown
            return

        # calculate fps
        current_time = time.time()
        fps = 1 / (current_time - self.last_render_time)
        self.last_render_time = current_time
        self.fps = (self.fps * 10 + fps) / 11

        # render scene
        mujoco.mjv_updateScene(self.model, data, self.scene_option, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(self.window)
        mujoco.mjr_render(self.viewport, self.scene, self.context)

        if self.game_state is not None:
            w, h = self.viewport.width, self.viewport.height
            bar_h_start = 30
            bar_h = 40
            y0, y1 = h - bar_h - bar_h_start, h - bar_h_start
            play_mode_bar_y0, play_mode_bar_y1 = y0, y0 - bar_h

            max_team_str_len = 20
            max_score_str_len = 5
            max_time_str_len = 5
            left_team_txt = self.game_state.left_team or 'Unknown'[:max_team_str_len]
            score_text = f'{self.game_state.left_score}:{self.game_state.right_score}'[:max_score_str_len]
            right_team_txt = (self.game_state.right_team or 'Unknown')[:max_team_str_len]
            mins = int(self.game_state.play_time // 60)
            secs = int(self.game_state.play_time % 60)
            time_text = f'{mins:02d}:{secs:02d}'[:max_time_str_len]
            play_mode_text = f'Playmode: {self.game_state.play_mode}'

            # fmt: off
            char_w    = 16
            box_w     = char_w * max_team_str_len
            mid_left  = 0.44 * w
            mid_right = 0.56 * w
            lx0, lx1  = mid_left  - box_w/2, mid_left  + box_w/2
            rx0, rx1  = mid_right - box_w/2, mid_right + box_w/2
            text_y    = int(h - bar_h_start - bar_h/2 - 8)

            sx0, sx1     = lx1, rx0

            time_box_w   = char_w * max_time_str_len
            padding      = 10
            tx0          = rx1 + padding
            tx1          = tx0 + time_box_w
            # fmt: on

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
                GL.glVertex2f(lx0 - 5, y0 - 5)
                GL.glVertex2f(tx1 + 5, y0 - 5)
                GL.glVertex2f(tx1 + 5, y1 + 5)
                GL.glVertex2f(lx0 - 5, y1 + 5)
                GL.glEnd()

                GL.glColor4f(0.2, 0.2, 0.2, 0.5)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(lx0 - 5, play_mode_bar_y0 - 5)
                GL.glVertex2f(tx1 + 5, play_mode_bar_y0 - 5)
                GL.glVertex2f(tx1 + 5, play_mode_bar_y1)
                GL.glVertex2f(lx0 - 5, play_mode_bar_y1)
                GL.glEnd()

                GL.glColor4f(0.0, 0.0, 0.8, 0.8)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(lx0, y0)
                GL.glVertex2f(lx1, y0)
                GL.glVertex2f(lx1, y1)
                GL.glVertex2f(lx0, y1)
                GL.glEnd()

                GL.glColor4f(0.8, 0.0, 0.0, 0.8)
                GL.glBegin(GL.GL_QUADS)
                GL.glVertex2f(rx0, y0)
                GL.glVertex2f(rx1, y0)
                GL.glVertex2f(rx1, y1)
                GL.glVertex2f(rx0, y1)
                GL.glEnd()

                GL.glPopMatrix()
                GL.glMatrixMode(GL.GL_PROJECTION)
                GL.glPopMatrix()

                GL.glPopAttrib()
                GL.glMatrixMode(GL.GL_MODELVIEW)

            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL,
                left_team_txt,
                self.context,
                (lx0 + ((max_team_str_len - len(left_team_txt)) / 2 * char_w)) / w,
                (text_y - 12) / h,
                1.0,
                1.0,
                1.0,
            )
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL,
                right_team_txt,
                self.context,
                (rx0 + ((max_team_str_len - len(right_team_txt)) / 2 * char_w)) / w,
                (text_y - 12) / h,
                1.0,
                1.0,
                1.0,
            )

            sx_center = sx0 + (sx1 - sx0) / 2
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL,
                score_text,
                self.context,
                (sx_center - (len(score_text) / 2 * char_w) + 5) / w,
                (text_y - 12) / h,
                1.0,
                1.0,
                1.0,
            )
            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL,
                time_text,
                self.context,
                (tx0 + ((max_time_str_len - len(time_text)) / 2 * char_w)) / w,
                (text_y - 12) / h,
                1.0,
                1.0,
                1.0,
            )

            mujoco.mjr_text(
                mujoco.mjtFont.mjFONT_NORMAL,
                play_mode_text,
                self.context,
                (lx0) / w,
                ((text_y - 12) - (bar_h)) / h,
                1.0,
                1.0,
                1.0,
            )

        if not self.hide_menu:
            overlays = self.create_overlays()
            for gridpos, [t1, t2] in overlays.items():
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, gridpos, self.viewport, t1, t2, self.context)

        glfw.swap_buffers(self.window)
        glfw.poll_events()

        self.frames += 1
        self.set_camera()

        if glfw.window_should_close(self.window):
            self.shutdown()

    def create_overlays(self) -> dict[Any, list[str]]:
        """Create overlays dictionary."""

        overlays: dict[Any, list[str]] = {}

        topleft = mujoco.mjtGridPos.mjGRID_TOPLEFT
        bottomright = mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT

        overlays[bottomright] = ['Framerate:', str(int(self.fps))]
        overlays[topleft] = ['', '']
        overlays[topleft][0] += 'Press K for "Kick-Off Left".\n'
        overlays[topleft][1] += '\n'
        overlays[topleft][0] += 'Press J for "Kick-Off Right".\n'
        overlays[topleft][1] += '\n'
        overlays[topleft][0] += 'Press B for "Drop-Ball".\n'
        overlays[topleft][1] += '\n'
        overlays[topleft][0] += 'Press H to hide the menu.\n'
        overlays[topleft][1] += '\n'
        overlays[topleft][0] += 'Press TAB to switch cameras.\n'
        overlays[topleft][1] += '\n'
        overlays[topleft][0] += 'Camera mode:\n'
        overlays[topleft][1] += self.camera_mode + '\n'

        return overlays

    def set_camera(self) -> None:
        """Set the active camera mode."""

        if self.camera_mode_target == 'static' and self.camera_mode != 'static':
            self.camera.fixedcamid = 0
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.trackbodyid = -1
            self.camera.distance = 15.0
            self.camera.elevation = -45.0
            self.camera.azimuth = 90.0

        if self.camera_mode_target == 'follow' and self.camera_mode != 'follow':
            self.camera.fixedcamid = -1
            self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.camera.trackbodyid = 0
            self.camera.distance = 3.5
            self.camera.elevation = 0.0
            self.camera.azimuth = 90.0

        self.camera_mode = self.camera_mode_target
