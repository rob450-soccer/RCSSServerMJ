import logging
import socket
import time
from math import degrees
from queue import Empty
from threading import Thread
from typing import TYPE_CHECKING, Any, Final, cast

import mujoco
import numpy as np

from rcsssmj.agent import AgentID
from rcsssmj.client.perception import AccelerometerPerception, GyroPerception, JointPerception, OrientationPerception, Perception, PositionPerception, TimePerception, TouchPerception
from rcsssmj.client.sim_client import SimClient, SimClientState
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.game.referee import SoccerReferee
from rcsssmj.monitor.monitor_client import MonitorClient
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.resources.spec_provider import ModelSpecProvider

if TYPE_CHECKING:
    from rcsssmj.client.action import SimAction
    from rcsssmj.monitor.sim_monitor import SimMonitor


logger = logging.getLogger(__name__)


class Server:
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

        self._spec_provider: ModelSpecProvider = ModelSpecProvider()
        """Mujoco model specification provider for loading models."""

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
            client.shutdown(no_wait=False)
        self._clients.clear()
        logger.info('Disconnected clients.')

        # shutdown active monitors
        for monitor in self._monitors:
            monitor.shutdown(no_wait=False)
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
            client = SimClient(conn)

            # TODO: synchronize threads when accessing client list
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
            monitor = MonitorClient(conn)

            # TODO: synchronize threads when accessing monitor list
            self._monitors.append(monitor)

        logger.info('Shutdown monitor listener thread.')
        self._monitor_sock.close()

    def _run_simulation(self) -> None:
        """Simulation main loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Starting Simulation loop.')

        # load soccer pitch spec
        spec = self._spec_provider.load_environment('soccer')
        if spec is None:
            return

        # prepare initial simulation model and data
        mj_model = spec.compile()
        mj_data = mujoco.MjData(mj_model)
        n_substeps = 5
        sim_timestep: float = mj_model.opt.timestep * n_substeps

        # create internal monitor
        if self.render:
            self._monitors.append(MujocoMonitor(mj_model, 2))

        needs_recompile: bool = False
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

            client_actions: list[SimAction] = []

            # TODO: synchronize threads when accessing client list
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

            # TODO: synchronize threads when accessing monitor list
            for monitor in self._monitors:
                if monitor.is_active():
                    active_monitors.append(monitor)
                else:
                    monitors_to_remove.append(monitor)

            # handle disconnected clients
            for client in disconnected_clients:
                # check if client has been activated before
                agent_id = client.get_id()
                if agent_id is not None:
                    logger.info('Team %s: Player %d left the game.', client.get_team_name(), client.get_player_no())

                    # remove client model from simulation
                    spec.detach_body(spec.body(agent_id.prefix + 'torso'))
                    needs_recompile = True

                    # remove agent from game
                    self.referee.handle_withdrawal(agent_id)

            # handle ready clients
            for client in ready_clients:
                # try to load the robot model requested by the client
                robot_spec = self._spec_provider.load_robot(client.get_model_name())
                if robot_spec is None:
                    # failed to load the requested model
                    client.shutdown()
                    clients_to_remove.append(client)
                    continue

                # request participation in game
                agent_id = self.referee.request_participation(client, robot_spec)
                if agent_id is None:
                    # invalid team side -> shutdown client
                    client.shutdown()
                    clients_to_remove.append(client)
                    continue

                logger.info('Team %s: Player #%2d joined the game %s.', client.get_team_name(), client.get_player_no(), client.get_addr())

                # append robot to world
                frame = spec.worldbody.add_frame()
                frame.attach_body(robot_spec.body('torso'), agent_id.prefix, '')
                needs_recompile = True

                client.activate(agent_id, robot_spec)
                activated_clients.append(client)
                active_clients.append(client)

            if needs_recompile:
                mj_model, mj_data = spec.recompile(mj_model, mj_data)
                needs_recompile = False

            # initialize newly activated players
            for client in activated_clients:
                self.referee.spawn_agent(cast(AgentID, client.get_id()), mj_data)

            # generate perceptions
            self._generate_perceptions(active_clients, mj_data)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # collect client actions and send perceptions
            for client in active_clients:
                # collect all pending actions
                action_queue = client.get_action_queue()
                try:
                    if self.sync_mode:
                        # wait for exactly one client action
                        client_actions += action_queue.get(timeout=10)
                    else:
                        # fetch all currently available actions
                        while action_queue.qsize() > 0:
                            client_actions += action_queue.get_nowait()
                except Empty:
                    if self.sync_mode:
                        # client took too long to answer -> kill it
                        logger.info('Team %s: Player %d did not respond for more than 10 seconds. Forcing player shutdown.', client.get_team_name(), client.get_player_no())
                        client.shutdown()
                        continue

                # send perceptions
                client.send_perceptions()

            # apply client actions
            for action in client_actions:
                action.perform(self.referee, mj_data)

            # progress simulation
            mujoco.mj_step(mj_model, mj_data, n_substeps)

            # apply monitor commands
            for monitor in active_monitors:
                command_queue = monitor.get_command_queue()
                try:
                    while command_queue.qsize() > 0:
                        command = command_queue.get_nowait()
                        command.perform(self.referee, mj_data)
                except Empty:
                    pass

            # call referee to judge the current simulation state and progress the game state
            self.referee.referee(mj_data)

            # update connected monitors
            for monitor in active_monitors:
                monitor.update(mj_model, mj_data)

            # TODO: log monitor message to simulator log
            # TODO: log client perceptions and actions to client logs

            # remove disconnected clients
            # TODO: synchronize threads when accessing client list
            for client in clients_to_remove:
                self._clients.remove(client)

            # remove inactive monitors
            # TODO: synchronize threads when accessing monitor list
            for monitor in monitors_to_remove:
                self._monitors.remove(monitor)

        logger.info('Simulation thread finished.')

    def _generate_perceptions(self, active_clients: list[SimClient], mj_data: Any) -> None:
        """Generate perceptions for active clients.

        Parameter
        ---------
        active_clients: list[SimClient]
            The list of clients considered as active in this simulation cycle.

        mj_data: mjData
            The current simulation state.
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

        # generate general perceptions equal for all clients
        sim_time_perception = TimePerception('now', trunc2(mj_data.time))
        game_state_perception = self.referee.generate_perception()

        # generate client specific perceptions
        for client in active_clients:
            client.add_perception(sim_time_perception)
            client.add_perception(game_state_perception)

            model_spec = cast(Any, client.get_model_spec())
            agent_id = cast(AgentID, client.get_id())
            prefix_length = len(agent_id.prefix)

            for sensor_spec in model_spec.sensors:
                sensor = mj_data.sensor(sensor_spec.name)
                sensor_name = sensor_spec.name[prefix_length:]

                if sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    client.add_perception(JointPerception(sensor_name, trunc2(degrees(sensor.data[0]))))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_GYRO:
                    rvx, rvy, rvz = trunc2_vec(np.degrees(sensor.data[0:3]))
                    client.add_perception(GyroPerception(sensor_name, rvx, rvy, rvz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                    ax, ay, az = trunc2_vec(sensor.data[0:3])
                    client.add_perception(AccelerometerPerception(sensor_name, ax, ay, az))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_TOUCH:
                    active = int(sensor.data[0])
                    client.add_perception(TouchPerception(sensor_name, active))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEQUAT:
                    qw, qx, qy, qz = trunc3_vec(sensor.data[0:4])
                    client.add_perception(OrientationPerception(sensor_name, qw, qx, qy, qz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEPOS:
                    px, py, pz = trunc3_vec(sensor.data[0:3])
                    client.add_perception(PositionPerception(sensor_name, px, py, pz))

                # TODO: Add perceptions for force, vision and hear

                else:
                    # sensor not supported...
                    pass
