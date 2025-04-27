import sys
import time
from itertools import cycle
from queue import Queue
from typing import Any

import glfw
import mujoco

from rcsssmj.monitor.commands import DropBallCommand, KickOffCommand, MonitorCommand
from rcsssmj.monitor.sim_monitor import SimMonitor


class MujocoMonitor(SimMonitor):
    """
    Internal monitor component.
    """

    def __init__(self, model: Any, dt: float) -> None:
        """
        Construct a new internal monitor viewer.
        """

        self.model = model
        self.dt = dt

        self.button_left: bool = False
        self.button_right: bool = False
        self.button_middle: bool = False
        self.last_x: int = 0
        self.last_y: int = 0
        self.frames: int = 0
        self.loop_count: float = 0.0
        self.target_render_time: float = 1 / 30.
        self.time_per_render = self.target_render_time

        self.command_queue: Queue[MonitorCommand] = Queue()
        self.run_speed_factor: float = 1.0
        self.paused: bool = False
        self.hide_menu: bool = False
        self.overlay: dict[Any, list[str]] = {}
        self.font_scale = 100

        glfw.init()
        glfw.window_hint(glfw.SCALE_TO_MONITOR, glfw.TRUE)

        primary_monitor = glfw.get_primary_monitor()
        video_mode = glfw.get_video_mode(primary_monitor)
        window_width, window_height = video_mode.size
        self.window = glfw.create_window(width=window_width, height=window_height, title="MuJoCo", monitor=None, share=None)
        glfw.make_context_current(self.window)

        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_scroll_callback(self.window, self.scroll)

        self.scene = mujoco.MjvScene(model, 1000)
        self.scene_option = mujoco.MjvOption()

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.all_camera_modes = ("static", "follow")
        self.camera_mode_iter = cycle(self.all_camera_modes)
        self.camera_mode = next(self.camera_mode_iter)
        self.camera_mode_target = self.camera_mode
        self.set_camera()

        framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(self.window)
        self.viewport = mujoco.MjrRect(0, 0, framebuffer_width, framebuffer_height)
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale(self.font_scale))

    def is_active(self) -> bool:
        """
        Check if the monitor is still active.
        """

        return True

    def get_command_queue(self) -> Queue[MonitorCommand]:
        """
        Return the command queue associated with this client.
        """

        return self.command_queue

    def shutdown(self, *, no_wait: bool = True) -> None:
        """
        Stop the monitor.
        """

        del no_wait  # signal unused parameter

        self.close()
        self.stop()

    def update(self, mj_model: Any, mj_data: Any) -> None:
        """
        Update the monitor state.
        """

        self.model = mj_model
        self.render(mj_data)

    def mouse_button(self, window: Any, button: Any, act: Any, mods: Any) -> None:
        """
        Update mouse button state.
        """

        self.button_left = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self.button_right = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        self.button_middle = glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS

        self.last_x, self.last_y = glfw.get_cursor_pos(self.window)

    def mouse_move(self, window: Any, x_pos: int, y_pos: int) -> None:
        """
        Handle mouse movement.
        """

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
        """
        Handle keyboard inputs.
        """

        if act != glfw.RELEASE:
            return

        if key == glfw.KEY_SPACE:
            self.paused = not self.paused
        elif key == glfw.KEY_H:
            self.hide_menu = not self.hide_menu
        elif key == glfw.KEY_TAB:
            self.camera_mode_target = next(self.camera_mode_iter)
        elif key == glfw.KEY_S:
            self.run_speed_factor /= 2.0
        elif key == glfw.KEY_F:
            self.run_speed_factor *= 2.0
        elif key == glfw.KEY_K:
            self.command_queue.put(KickOffCommand(0))
        elif key == glfw.KEY_J:
            self.command_queue.put(KickOffCommand(1))
        elif key == glfw.KEY_B:
            self.command_queue.put(DropBallCommand())

    def scroll(self, window: Any, x_offset: Any, y_offset: Any) -> None:
        """
        Handle scroll input.
        """

        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, 0.05 * y_offset, self.scene, self.camera)

    def render(self, data: Any) -> None:
        """
        Render function.
        """

        def render_inner_loop(self: MujocoMonitor) -> None:
            self.create_overlay()
            render_start = time.time()

            mujoco.mjv_updateScene(self.model, data, self.scene_option, None, self.camera,
                                   mujoco.mjtCatBit.mjCAT_ALL,
                                   self.scene)
            self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(self.window)
            mujoco.mjr_render(self.viewport, self.scene, self.context)

            if not self.hide_menu:
                for gridpos, [t1, t2] in self.overlay.items():
                    mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_SHADOW, gridpos, self.viewport, t1, t2, self.context)

            glfw.swap_buffers(self.window)
            glfw.poll_events()

            self.frames += 1
            self.overlay.clear()

            if glfw.window_should_close(self.window):
                self.stop()
                sys.exit(0)

            time.sleep(max(0, self.target_render_time - (time.time() - render_start) - 0.0002))
            self.time_per_render = time.time() - render_start

        if self.paused:
            while self.paused:
                render_inner_loop(self)

        self.loop_count += self.dt / (self.time_per_render * self.run_speed_factor)
        while self.loop_count > 0:
            render_inner_loop(self)
            self.set_camera()
            self.loop_count -= 1

    def stop(self) -> None:
        """
        Stop monitor.
        """

        glfw.destroy_window(self.window)

    def close(self) -> None:
        """
        Close monitor window.
        """

        glfw.set_window_should_close(self.window, True)

    def create_overlay(self) -> None:
        """
        Set overlay entries.
        """

        topleft = mujoco.mjtGridPos.mjGRID_TOPLEFT
        bottomright = mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT

        self.overlay[bottomright] = ["Framerate:", str(int(1/self.time_per_render * self.run_speed_factor))]
        self.overlay[topleft] = ["", ""]
        self.overlay[topleft][0] += 'Press K for "Kick-Off Left".\n'
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += 'Press J for "Kick-Off Right".\n'
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += 'Press B for "Drop-Ball".\n'
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += "Press SPACE to pause.\n"
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += "Press H to hide the menu.\n"
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += "Press TAB to switch cameras.\n"
        self.overlay[topleft][1] += "\n"
        self.overlay[topleft][0] += "Camera mode:\n"
        self.overlay[topleft][1] += self.camera_mode+"\n"
        self.overlay[topleft][0] += f"Run speed = {self.run_speed_factor:.3f} x real time"
        self.overlay[topleft][1] += "[S]lower, [F]aster"

    def set_camera(self) -> None:
        """
        Set the active camera mode.
        """

        if self.camera_mode_target == "static" and self.camera_mode != "static":
                self.camera.fixedcamid = 0
                self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
                self.camera.trackbodyid = -1
                self.camera.distance = 15.0
                self.camera.elevation = -45.0
                self.camera.azimuth = 90.0

        if self.camera_mode_target == "follow" and self.camera_mode != "follow":
                self.camera.fixedcamid = -1
                self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self.camera.trackbodyid = 0
                self.camera.distance = 3.5
                self.camera.elevation = 0.0
                self.camera.azimuth = 90.0

        self.camera_mode = self.camera_mode_target
