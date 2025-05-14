import logging
import socket
import time
from collections.abc import Sequence
from queue import Empty
from threading import Lock, Thread
from typing import Any, Final, cast

import mujoco
import numpy as np

from rcsssmj.agent import AgentID
from rcsssmj.client.action import SimAction
from rcsssmj.client.perception import (
    AccelerometerPerception,
    AgentDetection,
    GyroPerception,
    JointStatePerception,
    ObjectDetection,
    OrientationPerception,
    Perception,
    PObjectDetection,
    PositionPerception,
    TimePerception,
    TouchPerception,
    VisionPerception,
)
from rcsssmj.client.sim_client import SimClient, SimClientState, TCPSimClient
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.game.referee import SoccerReferee
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.monitor.sim_monitor import SimMonitor, SimMonitorState, TCPSimMonitor
from rcsssmj.resources.spec_provider import ModelSpecProvider

logger = logging.getLogger(__name__)


class BaseSimulation:
    """Base class for simulations."""

    def __init__(
        self,
        *,
        spec_provider: ModelSpecProvider | None = None,
        n_substeps: int = 4,
        vision_interval: int = 4,
    ) -> None:
        """Construct a new simulation.

        Parameter
        ---------
        substeps: int, default=5
            The number of simulation substeps between the agent update cycles.

        vision_interval: int, default=4
            The interval in which vision perception will be generated.
        """

        self._spec_provider: ModelSpecProvider = ModelSpecProvider() if spec_provider is None else spec_provider
        """Mujoco model specification provider for loading models."""

        self.n_substeps: Final[int] = n_substeps
        """The number of simulation substeps."""

        self.vision_interval: Final[int] = vision_interval
        """The interval (in simulation cycles) in which the vision perception is generated."""

        self._frame_id: int = 0
        """The current simulation frame number."""

        self._mj_spec: Any = None
        """The mujoco simulation model specification."""

        self._mj_model: Any = None
        """The mujoco simulation model."""

        self._mj_data: Any = None
        """The mujoco simulation data array."""

        self._world_markers: Sequence[tuple[str, str]] = []
        """The sequence of world markers used for generating vision perceptions."""

    @property
    def frame_id(self) -> int:
        """The simulation frame id."""

        return self._frame_id

    @property
    def mj_spec(self) -> Any:
        """The mujoco simulation model specification."""

        return self._mj_spec

    @property
    def mj_model(self) -> Any:
        """The mujoco simulation model."""

        return self._mj_model

    @property
    def mj_data(self) -> Any:
        """The mujoco simulation data array."""

        return self._mj_data

    def _create_world(self, referee: SoccerReferee) -> bool:
        """(Re-)initialize the game and create a new simulation world environment."""

        # initialize game and create game world environment
        self._mj_spec = referee.init_game(self._spec_provider)
        if self._mj_spec is None:
            logger.warning('Failed to initialize game.')
            return False

        # prepare initial simulation model and data
        self._mj_model = self._mj_spec.compile()
        self._mj_data = mujoco.MjData(self._mj_model)

        # extract visible object markers of world
        self._world_markers = [(site.name, site.name[:-10]) for site in self._mj_spec.sites if site.name.endswith('-vismarker')]

        # reset frame id
        self._frame_id = 0

        return True

    def _activate_client(self, client: SimClient, referee: SoccerReferee) -> bool:
        """Try to activate the given client.

        Parameter
        ---------
        client: SimClient
            The simulation cleint to activate.

        referee: SoccerReferee
            The referee instance.
        """

        # try to load the robot model requested by the client
        robot_spec = self._spec_provider.load_robot(client.get_model_name())
        if robot_spec is None:
            # failed to load the requested model --> report failure
            return False

        # request participation in game
        agent_id = referee.request_participation(client, robot_spec)
        if agent_id is None:
            # invalid team side -> report failure
            return False

        # append robot to world
        frame = self._mj_spec.worldbody.add_frame()
        frame.attach_body(robot_spec.body('torso'), agent_id.prefix, '')

        client.activate(agent_id, robot_spec)

        return True

    def _deactivate_client(self, client: SimClient, referee: SoccerReferee) -> bool:
        """Deactivate the given client instance.

        Parameter
        ---------
        client: SimClient
            The simulation cleint to activate.

        referee: SoccerReferee
            The referee instance.
        """

        # check if client has been activated before
        agent_id = client.get_id()

        if agent_id is not None:
            # remove client model from simulation
            self._mj_spec.detach_body(self._mj_spec.body(agent_id.prefix + 'torso'))

            # delete various components manually, as they are not automatically removed again when the root body is detached
            # Note:
            # Not sure if this is intentional behavior or a bug in mujoco.
            # It's also not clear what components need to be deleted separately.
            # The code below so far prevents any follow-up exceptions when re-attaching the same model again.
            # But at the moment, there is no guarantee that there will be no components left in the spec that may cause some trouble at some point.
            def del_els(el_list: list[Any]) -> None:
                for el in el_list:
                    el.delete()

            model_spec = cast(Any, client.get_model_spec())
            # del_els(model_spec.cameras)
            # del_els(model_spec.geoms)
            # del_els(model_spec.lights)
            del_els(model_spec.materials)
            del_els(model_spec.meshes)
            del_els(model_spec.sites)
            del_els(model_spec.texts)
            del_els(model_spec.textures)

            # remove agent from game
            referee.handle_withdrawal(agent_id)

            return True

        return False

    def _collect_actions(self, active_clients: Sequence[SimClient], *, block: bool = False, timeout: float = 5) -> list[SimAction]:
        """Collect the actions from all active clients.

        Parameter
        ---------
        active_clients: Sequence[SimClient]
            The list of active clients.

        block: bool, default=False
            Wait for client actions to arrive.

        timeout: float, default=10
            The time to wait for client actions to arrive. After this time, the client is considered inactive and will be shutdown.
            If timeout is a negative number, it will wait forever.
        """

        client_actions: list[SimAction] = []

        # collect client actions and send perceptions
        for client in active_clients:
            # collect all pending actions
            action_queue = client.get_action_queue()
            try:
                if block:
                    # wait for exactly one client action
                    client_actions += action_queue.get(timeout=timeout)
                else:
                    # fetch all currently available actions
                    while action_queue.qsize() > 0:
                        client_actions += action_queue.get_nowait()
            except Empty:
                if block:
                    # client took too long to answer -> kill it
                    logger.info('Team %s: Player %d did not respond for more than %.3f seconds. Forcing player shutdown.', client.get_team_name(), client.get_player_no(), timeout)
                    client.shutdown()
                    continue

        return client_actions

    def _collect_commands(self, active_monitors: Sequence[SimMonitor]) -> list[MonitorCommand]:
        """Collect the commands from all active monitors.

        Parameter
        ---------
        active_monitors: Sequence[SimMonitor]
            The list of active monitors.
        """

        monitor_commands: list[MonitorCommand] = []

        for monitor in active_monitors:
            command_queue = monitor.get_command_queue()
            try:
                while command_queue.qsize() > 0:
                    monitor_commands.append(command_queue.get_nowait())
            except Empty:
                pass

        return monitor_commands

    def _step(self, client_actions: Sequence[SimAction], monitor_commands: Sequence[MonitorCommand], referee: SoccerReferee) -> None:
        """Perform a simulation step.

        Parameter
        ---------
        active_monitors: Sequence[SimMonitor]
            The list of active monitors.
        """

        # apply client actions
        for action in client_actions:
            action.perform(referee, self._mj_data)

        # progress simulation
        mujoco.mj_step(self._mj_model, self._mj_data, self.n_substeps)

        # apply monitor commands
        for command in monitor_commands:
            command.perform(referee, self._mj_data)

        # call referee to judge the current simulation state and progress the game state
        referee.referee(self._mj_data)

        # increment frame id
        self._frame_id += 1

    def _generate_perceptions(self, active_clients: Sequence[SimClient], referee: SoccerReferee, *, gen_vision: bool | None = None) -> None:
        """Generate perceptions for active clients.

        Parameter
        ---------
        active_clients: Sequence[SimClient]
            The list of clients considered as active in this simulation cycle.

        referee: SoccerReferee
            The referee instance.

        gen_vision: bool, default=False
            Generate vision perception.
        """

        def trunc2(val: float) -> float:
            """Limit the given value to two digits."""
            return int(val * 100) / 100.0

        def trunc2_vec(vec: Any) -> Any:
            """Limit the given vector to two digits."""
            return np.trunc(vec * 100) / 100.0

        def trunc3_vec(vec: Any) -> Any:
            """Limit the given vector to three digits."""
            return np.trunc(vec * 1000) / 1000.0

        # default to vision interval if gen_vision parameter is not specified
        if gen_vision is None:
            gen_vision = self._frame_id % self.vision_interval == 0

        # generate general perceptions equal for all clients
        sim_time_perception = TimePerception('now', trunc2(self._mj_data.time))
        game_state_perception = referee.generate_perception()

        if gen_vision:
            # collect visible markers
            n_world_markers = len(self._world_markers)
            obj_markers = list(self._world_markers)
            for client in active_clients:
                obj_markers.extend(client.get_model_markers())

            # extract visible object positions
            n_markers = len(obj_markers)
            obj_pos = np.zeros((3, n_markers), dtype=np.float64)
            for idx, site in enumerate(obj_markers):
                obj_pos[:, idx] = self._mj_data.site(site[0]).xpos.astype(np.float64)

        # generate client specific perceptions
        for client in active_clients:
            joint_names: list[str] = []
            joint_axs: list[float] = []
            joint_vxs: list[float] = []
            client_perceptions: list[Perception] = [sim_time_perception, game_state_perception]

            model_spec = cast(Any, client.get_model_spec())
            agent_id = cast(AgentID, client.get_id())
            prefix_length = len(agent_id.prefix)

            for sensor_spec in model_spec.sensors:
                sensor = self._mj_data.sensor(sensor_spec.name)
                sensor_name = sensor_spec.name[prefix_length:]

                if sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    joint_names.append(sensor_name)
                    joint_axs.append(sensor.data[0])
                
                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTVEL:
                    joint_vxs.append(sensor.data[0])

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_GYRO:
                    rvx, rvy, rvz = trunc2_vec(np.degrees(sensor.data[0:3]))
                    client_perceptions.append(GyroPerception(sensor_name, rvx, rvy, rvz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                    ax, ay, az = trunc2_vec(sensor.data[0:3])
                    client_perceptions.append(AccelerometerPerception(sensor_name, ax, ay, az))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_TOUCH:
                    active = int(sensor.data[0])
                    client_perceptions.append(TouchPerception(sensor_name, active))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEQUAT:
                    qw, qx, qy, qz = trunc3_vec(sensor.data[0:4])
                    client_perceptions.append(OrientationPerception(sensor_name, qw, qx, qy, qz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEPOS:
                    px, py, pz = trunc3_vec(sensor.data[0:3])
                    client_perceptions.append(PositionPerception(sensor_name, px, py, pz))

                # TODO: Add perceptions for force and hear

                else:
                    # sensor not supported...
                    pass

            # joint state perception
            if joint_names:
                client_perceptions.append(JointStatePerception(joint_names, trunc2_vec(np.degrees(joint_axs)), trunc2_vec(np.degrees(joint_vxs))))

            # ideal camera sensor-pipeline
            if gen_vision:
                # fetch camera sensor site
                camera_site = self._mj_data.site(agent_id.prefix + 'camera')

                if camera_site is not None:
                    # fetch pose of camera site of robot model
                    camera_pos = camera_site.xpos.astype(np.float64)
                    camera_rot = camera_site.xmat.astype(np.float64).reshape((3, 3))

                    # transform detectable obj positions to camera frame
                    local_obj_pos = np.matmul(camera_rot.T, obj_pos - camera_pos[:, np.newaxis])

                    # transform local positions into polar coordinates
                    anzimuths = trunc2_vec(np.degrees(np.atan2(local_obj_pos[1], local_obj_pos[0])))
                    distances = np.linalg.norm(local_obj_pos, axis=0)
                    elevations = trunc2_vec(np.degrees(np.asin(local_obj_pos[2] / distances)))
                    distances = trunc2_vec(distances)

                    # TODO: Apply sensor noise

                    # check object coordinates for horizontal and vertical view range
                    half_horizontal_range = 60
                    half_vertical_range = 60
                    obj_visibility = (anzimuths >= -half_horizontal_range) & (anzimuths <= half_horizontal_range) & (elevations >= -half_vertical_range) & (elevations <= half_vertical_range)

                    # extract simple world object detections
                    obj_detections: list[PObjectDetection] = [ObjectDetection(obj_markers[i][1], anzimuths[i], elevations[i], distances[i]) for i in range(n_world_markers) if obj_visibility[i]]

                    # extract agent object detections
                    idx = n_world_markers
                    for agent in active_clients:
                        n_agent_markers = len(agent.get_model_markers())
                        agent_detections = [ObjectDetection(obj_markers[i][1], anzimuths[i], elevations[i], distances[i]) for i in range(idx, idx + n_agent_markers) if obj_visibility[i]]
                        if agent_detections:
                            obj_detections.append(AgentDetection('P', agent.get_team_name(), agent.get_player_no(), agent_detections))

                        idx += n_agent_markers

                    client_perceptions.append(VisionPerception('See', obj_detections))

            # forward generated perceptions to client instance
            client.set_perceptions(client_perceptions)


class SimServer(BaseSimulation):
    """The simulation server component.

    The simulation server is the core server component, responsible for running the central simulation lopp as well as managing client and monitor connections / communication.
    Game specific logic is encapsulated in a referee instance, which is incorporated into the simulation loop.

    By default, the simulation server runs in a competition setup mode.
    This means that it will try to simulate in real time and will not wait for client actions to arrive before the next simulation cycle.
    In this scenario, connected clients are responsible for managing their resources and performance to respond in time (as it is the case for a real robot, too).

    However, the server also offers a set of flags with which you can setup the server in a training / evaluation mode.
    Use the `sync_mode` flag to tell the server to wait for an action response of all active agents before simulating the next simulation cycle.
    When disabling the `real_time` flag, the server will not wait between simulation cycles to simulate a real-time scenario.
    Instead, it will directly progress to simulating the next simulation cycle - aka run "as-fast-as-possible".
    When disabling the `real_time` flag, it is adviced to activate sync mode, too, as otherwise there will be some severe desync between server and client processes.

    At this point in time, the simulation server comes with a built-in mujoco viewer as internal monitor, which will be started by default.
    Note that due to the Python GIL and the fact that the mujoco viewer is designed to run synchronously, rendering will impact the real-time capability of the simulation in certain scenarios.
    You can disable the internal monitor component by setting the `render` flag to `False`.
    """

    def __init__(
        self,
        referee: SoccerReferee,
        host: str = '127.0.0.1',
        client_port: int = 60000,
        monitor_port: int = 60001,
        *,
        sync_mode: bool = False,
        real_time: bool = True,
        render: bool = True,
    ) -> None:
        """Construct a new simulation sever.

        Parameter
        ---------
        host: str
            The server host address.

        client_port: int, default=60000
            The port on which to listen for incoming client connections.

        monitor_port: int, default=60001
            The port on which to listen for incoming monitor connections.

        sync_mode: bool, default=False
            Flag specifying if the server should run in sync-mode.
            In sync-mode (True), the server will waiting in each simulation cycle until all actions of all active agents arrived before simulating the next cycle.
            If sync-mode is disabled (default, False), then the server will not wait for any connected agents and simply process the actions that arrived in time for the next simulation cycle.

        real_time: bool, default=True
            Flag specifying if the server should run in real-time mode (default, True) or as-fast-as-possible (False).

        render: bool, default=True
            Flag for enabling (default, True) or disabling (False) the internal monitor viewer.
        """

        super().__init__()

        self.referee: Final[SoccerReferee] = referee
        """The game referee responsible for managing the game aspect of the simulation."""

        self.host: Final[str] = host
        """The server host address."""

        self.client_port: Final[int] = client_port
        """The port on which to listen for incoming client connections."""

        self.monitor_port: Final[int] = monitor_port
        """The port on which to listen for incoming monitor connections."""

        self.sync_mode: Final[bool] = sync_mode
        """Flag for enabling / disabling sync mode."""

        self.real_time: Final[bool] = real_time
        """Flag for enabling / disabling real-time mode."""

        self.render: Final[bool] = render
        """Flag for enabling / disabling the internal mujoco viewer monitor."""

        self._clients: list[SimClient] = []
        """The list of connected clients."""

        self._monitors: list[SimMonitor] = []
        """The list of connected monitors."""

        self._client_sock: socket.socket | None = None
        """The socket for listening for incoming client connections (only present after the server has beed started)."""

        self._monitor_sock: socket.socket | None = None
        """The socket for listening for incoming monitor connections (only present after the server has beed started)."""

        self._shutdown: bool = True
        """Flag indicating a shutdown request, causing the simulation server to shutdown."""

        self._mutex: Lock = Lock()
        """Mutex for synchronizing simulation threads."""

    def run(self) -> None:
        """
        Run simulation server.
        """

        if self._client_sock is not None or self._monitor_sock is not None:
            # a simulation is already running...
            raise RuntimeError

        # 1. SETUP: Setup sockets and start server threads
        logger.info('Starting server...')
        self._shutdown = False

        # setup client socket
        self._client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._client_sock.bind((self.host, self.client_port))
        self._client_sock.listen(5)

        # setup monitor socket
        try:
            self._monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._monitor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._monitor_sock.bind((self.host, self.monitor_port))
            self._monitor_sock.listen(5)
        except ConnectionError:
            self._client_sock.shutdown(socket.SHUT_RDWR)
            self._client_sock.close()
            raise

        # create simulator threads
        sim_thread = Thread(target=self._run_simulation, name='sim_loop')
        client_listener_thread = Thread(target=self._listen_for_clients, name='client_connections_listener')
        monitor_listener_thread = Thread(target=self._listen_for_monitors, name='monitor_connections_listener')

        # start simulator threads
        client_listener_thread.start()
        monitor_listener_thread.start()
        sim_thread.start()

        logger.info('Starting server... DONE!')

        # 2. RUN: Wait until simulation thread finished
        sim_thread.join()  # run simulation loop in separate thread to isolate exceptions and allow the main thread to clean up

        # 3. CLEANUP: Shutdown everything and wait for socket threads to finish
        logger.info('Shutting down Server...')
        self._shutdown = True

        # shutdown client and monitor sockets
        try:
            self._client_sock.shutdown(socket.SHUT_RDWR)
        except Exception:  # noqa: BLE001
            logger.debug('ERROR while shutting down client socket!', exc_info=True)

        try:
            self._monitor_sock.shutdown(socket.SHUT_RDWR)
        except Exception:  # noqa: BLE001
            logger.debug('ERROR while shutting down monitor socket!', exc_info=True)

        # wait for client and monitor connection listener threads to finish
        client_listener_thread.join()
        monitor_listener_thread.join()

        # shutdown active clients
        for client in self._clients:
            client.shutdown(wait=True)
        self._clients.clear()
        logger.info('Disconnected clients.')

        # shutdown active monitors
        for monitor in self._monitors:
            monitor.shutdown(wait=True)
        self._monitors.clear()
        logger.info('Disconnected monitors.')

        # cleanup socket refs
        self._client_sock = None
        self._monitor_sock = None

        logger.info('Shutting down server... DONE!')

    def shutdown(self) -> None:
        """
        Request server shutdown.
        """

        self._shutdown = True

        logger.info('Shutdown requested.')

    def _listen_for_clients(self) -> None:
        """Wait for incoming client connections.

        Note: This method is executed by the client listener thread - don't call it independently!
        """

        if self._client_sock is None:
            return

        logger.info('Listening for client connections on %s:%d', self.host, self.client_port)
        while not self._shutdown:
            try:
                sock, addr = self._client_sock.accept()
            except Exception:  # noqa: BLE001
                self._shutdown = True
                break

            logger.info('New client connection: %s.', addr)

            conn = TCPLPMConnection(sock, addr)
            client = TCPSimClient(conn)

            with self._mutex:
                self._clients.append(client)

        logger.info('Shutdown client listener thread.')
        self._client_sock.close()

    def _listen_for_monitors(self) -> None:
        """Wait for incoming monitor connections.

        Note: This method is executed by the monitor listener thread - don't call it independently!
        """

        if self._monitor_sock is None:
            return

        logger.info('Listening for monitor connections on %s:%d', self.host, self.monitor_port)
        while not self._shutdown:
            try:
                sock, addr = self._monitor_sock.accept()
            except Exception:  # noqa: BLE001
                self._shutdown = True
                break

            logger.info('New monitor connection: %s.', addr)

            conn = TCPLPMConnection(sock, addr)
            monitor = TCPSimMonitor(conn)

            with self._mutex:
                self._monitors.append(monitor)

        logger.info('Shutdown monitor listener thread.')
        self._monitor_sock.close()

    def _run_simulation(self) -> None:
        """Simulation main loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Starting Simulation loop.')

        # create simulation world
        if not self._create_world(self.referee):
            return

        # create internal monitor
        if self.render:
            self._monitors.append(MujocoMonitor(self._mj_model, 2))

        needs_recompile: bool = False
        sim_timestep: float = self._mj_model.opt.timestep * self.n_substeps
        cycle_start: float = time.time() - sim_timestep

        # simulation loop
        while not self._shutdown:
            # filter clients / monitors by state, as their state may change during this simulation step
            # this also simplifies client / monitor removal from the central client / monitor lists
            ready_clients: list[SimClient] = []
            active_clients: list[SimClient] = []
            disconnected_clients: list[SimClient] = []
            clients_to_remove: list[SimClient] = []
            activated_clients: list[SimClient] = []

            active_monitors: list[SimMonitor] = []
            monitors_to_remove: list[SimMonitor] = []

            with self._mutex:
                for client in self._clients:
                    if client.get_state() == SimClientState.READY:
                        ready_clients.append(client)
                    if client.get_state() == SimClientState.ACTIVE:
                        active_clients.append(client)
                    if client.get_state() == SimClientState.DISCONNECTED:
                        disconnected_clients.append(client)
                        clients_to_remove.append(client)
                    else:
                        # CONNECTED state not relevant here
                        pass

                for monitor in self._monitors:
                    if monitor.get_state() == SimMonitorState.CONNECTED:
                        active_monitors.append(monitor)
                    else:
                        monitors_to_remove.append(monitor)

            # handle disconnected clients
            for client in disconnected_clients:
                if self._deactivate_client(client, self.referee):
                    logger.info('Player %s left the game.', client)
                    needs_recompile = True

            # handle ready clients
            for client in ready_clients:
                if self._activate_client(client, self.referee):
                    logger.info('Player %s joined the game.', client)
                    needs_recompile = True
                    activated_clients.append(client)
                    active_clients.append(client)
                else:
                    # failed to activate client --> shutdown and remove client
                    logger.info('Failed to activete player %s. Disconnecting again.', client)
                    client.shutdown()
                    clients_to_remove.append(client)

            if needs_recompile:
                self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
                needs_recompile = False

            # initialize newly activated players
            if activated_clients:
                for client in activated_clients:
                    self.referee.spawn_agent(cast(AgentID, client.get_id()), self._mj_data)

                # calculate forward kinematics / dynamics of newly added robot models (without progrssing the time)
                mujoco.mj_forward(self._mj_model, self._mj_data)

            # generate perceptions
            self._generate_perceptions(active_clients, self.referee)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # collect client actions
            # Note: This has to be done separate to and before sending perceptions to clients to prevent fetching new actions that arrived while still sending perceptions.
            client_actions = self._collect_actions(active_clients, block=self.sync_mode)

            # send perceptions
            for client in active_clients:
                client.send_perceptions()

            # collect monitor commands
            monitor_commands = self._collect_commands(active_monitors)

            # progress simulation
            self._step(client_actions, monitor_commands, self.referee)

            # update connected monitors
            for monitor in active_monitors:
                monitor.update(self._mj_model, self._mj_data, self._frame_id)

            # TODO: log monitor message to simulator log
            # TODO: log client perceptions and actions to client logs

            # remove disconnected clients
            with self._mutex:
                for client in clients_to_remove:
                    self._clients.remove(client)

            # remove inactive monitors
            with self._mutex:
                for monitor in monitors_to_remove:
                    self._monitors.remove(monitor)

        logger.info('Simulation thread finished.')


class ManagedSim(BaseSimulation):
    """A simulation without the server component, running in sync with a single client, managed externally."""

    def __init__(self) -> None:
        """Construct a new training simulation."""

        super().__init__()

        self._referee: SoccerReferee | None = None
        """The game referee."""

        self._clients: Sequence[SimClient] = []
        """The list of active clients."""

        self._client_actions: Sequence[SimAction] = []
        """The list of buffered client actions for the next simulation cycle."""

    def get_client_timestep(self) -> float:
        """Return the client timestep."""

        return 0 if self.mj_model is None else self.mj_model.opt.timestep * self.n_substeps

    def reset(self, clients: SimClient | Sequence[SimClient], referee: SoccerReferee) -> None:
        """Reset the simulation for the given client and referee.

        Parameter
        ---------
        clients: SimClient | Sequence[SimClient]
            The (learning) client instance and possibly further clients executed synchronously with the simulation.

        referee: SoccerReferee
            The referee to use for judging the game.
        """

        # shutdown existing clients
        for client in self._clients:
            client.shutdown()

        # reset simulation state
        self._referee = referee
        self._clients = clients if isinstance(clients, Sequence) else [clients]
        self._client_actions = []

        # create a new world
        if not self._create_world(self._referee):
            raise RuntimeError

        # activate clients
        for client in self._clients:
            if not self._activate_client(client, self._referee):
                raise RuntimeError

        # compile modified spec
        self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)

        # spawn agents
        for client in self._clients:
            self._referee.spawn_agent(cast(AgentID, client.get_id()), self._mj_data)

        # initialize sim data and client perceptions
        self.forward()

    def forward(self) -> None:
        """
        Recalculate forward kinematics / dynamics and regenerate agent perceptions.

        This method does not perform a simulation step.
        It only recalculates the forward kinematics / dynamics and regenerates agent perceptions accordingly.
        Use it to refresh the simulator state after externally manipulating the mj_data array.
        """

        if self._referee is None or self._mj_data is None:
            raise RuntimeError

        # calculate forward kinematics / dynamics in case the use modified the data
        mujoco.mj_forward(self._mj_model, self._mj_data)

        # generate perceptions
        self._generate_perceptions(self._clients, self._referee)

    def step(self, commands: Sequence[MonitorCommand] = []) -> None:
        """Perform a simulation step.

        Parameter
        ---------
        commands: Sequence[MonitorCommand], default=[]
            The sequence of monitor commands to forward to the referee for this simulation cycle.
        """

        if self._referee is None or self._mj_data is None:
            raise RuntimeError

        # send perceptions
        for client in self._clients:
            client.send_perceptions()

        # progress simulation
        self._step(self._client_actions, commands, self._referee)

        # generate perceptions
        self._generate_perceptions(self._clients, self._referee)

        # buffer actions from client
        self._client_actions = self._collect_actions(self._clients)
