import logging
import socket
import time
from threading import Thread
from typing import Final

from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.simulation import BaseSimulation

logger = logging.getLogger(__name__)


class SimServer:
    """The simulation server component.

    The simulation server is the core server component, responsible for running the central simulation loop as well as managing client and monitor connections / communication.
    Game specific logic is encapsulated in a referee instance, which is incorporated into the simulation loop.

    By default, the simulation server runs in a competition setup mode.
    This means that it will try to simulate in real time and will not wait for client actions to arrive before the next simulation cycle.
    In this scenario, connected clients are responsible for managing their resources and performance to respond in time (as it is the case for a real robot, too).

    However, the server also offers a set of flags with which you can setup the server in a training / evaluation mode.
    Use the `sync_mode` flag to tell the server to wait for an action response of all active agents before simulating the next simulation cycle.
    When disabling the `real_time` flag, the server will not wait between simulation cycles to simulate a real-time scenario.
    Instead, it will directly progress to simulating the next simulation cycle - aka run "as-fast-as-possible".
    When disabling the `real_time` flag, it is advised to activate sync mode, too, as otherwise there will be some severe desync between server and client processes.

    At this point in time, the simulation server comes with a built-in mujoco viewer as internal monitor, which will be started by default.
    Note that due to the Python GIL and the fact that the mujoco viewer is designed to run synchronously, rendering will impact the real-time capability of the simulation in certain scenarios.
    You can disable the internal monitor component by setting the `render` flag to `False`.
    """

    def __init__(
        self,
        sim: BaseSimulation,
        host: str = '127.0.0.1',
        client_port: int = 60000,
        monitor_port: int = 60001,
        *,
        sequential_mode: bool = False,
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

        sequential_mode: bool, default=False
            Flag for selecting sequential or parallel simulation update loop.

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

        self.sim: Final[BaseSimulation] = sim
        """The simulation to run."""

        self.host: Final[str] = host
        """The server host address."""

        self.client_port: Final[int] = client_port
        """The port on which to listen for incoming client connections."""

        self.monitor_port: Final[int] = monitor_port
        """The port on which to listen for incoming monitor connections."""

        self.sequential_mode: Final[bool] = sequential_mode
        """Flag for enabling / disabling sequential mode."""

        self.sync_mode: Final[bool] = sync_mode or sequential_mode or not real_time
        """Flag for enabling / disabling sync mode."""

        self.real_time: Final[bool] = real_time
        """Flag for enabling / disabling real-time mode."""

        self.render: Final[bool] = render
        """Flag for enabling / disabling the internal mujoco viewer monitor."""

        self._client_sock: socket.socket | None = None
        """The socket for listening for incoming client connections (only present after the server has been started)."""

        self._monitor_sock: socket.socket | None = None
        """The socket for listening for incoming monitor connections (only present after the server has been started)."""

        self._shutdown: bool = True
        """Flag indicating a shutdown request, causing the simulation server to shutdown."""

    def run(self) -> None:
        """Run simulation server."""

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

        # shutdown simulation (remove active clients and monitors)
        self.sim.shutdown(wait=True)
        logger.info('Shutdown simulation (disconnected clients and monitors).')

        # cleanup socket refs
        self._client_sock = None
        self._monitor_sock = None

        logger.info('Shutting down server... DONE!')

    def shutdown(self) -> None:
        """Request server shutdown."""

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
            self.sim.register_clients(conn)

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
            self.sim.register_monitors(conn)

        logger.info('Shutdown monitor listener thread.')
        self._monitor_sock.close()

    def _run_simulation(self) -> None:
        """Simulation main loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Starting Simulation loop.')

        # create simulation world
        if not self.sim.init():
            return

        # create internal monitor
        if self.render:
            self.sim.register_monitors(MujocoMonitor(self.sim.mj_model, 2))

        # run simulation update loop
        if self.sequential_mode:
            self._sequential_update_loop()
        else:
            self._parallel_update_loop()

        logger.info('Simulation thread finished.')

    def _parallel_update_loop(self) -> None:
        """Parallel simulation update loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Running a parallel simulation update loop.')

        sim_timestep: float = self.sim.mj_model.opt.timestep * self.sim.n_substeps
        cycle_start: float = time.time() - sim_timestep

        # parallel simulation update loop
        while not self._shutdown:
            # filter clients / monitors by state, as their state may change during this simulation step
            # this also simplifies client / monitor removal from the central client / monitor lists
            _, ready_clients, active_clients, disconnected_clients = self.sim.filter_clients()
            active_monitors, monitors_to_remove = self.sim.filter_monitors()

            # handle disconnected clients
            self.sim.deactivate_clients(disconnected_clients)

            # handle ready clients
            activated_clients, deactivated_clients = self.sim.activate_clients(ready_clients)

            # generate perceptions
            self.sim.generate_perceptions(active_clients + activated_clients)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # collect client actions
            # Note: Actions need to be collected before sending perceptions to clients in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            client_actions = self.sim.collect_actions(active_clients, block=self.sync_mode)

            # send perceptions
            self.sim.send_perceptions(active_clients + activated_clients)

            # collect monitor commands
            monitor_commands = self.sim.collect_commands(active_monitors)

            # progress simulation
            self.sim.step(client_actions, monitor_commands)

            # update connected monitors
            self.sim.update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log client perceptions and actions to client logs

            # remove disconnected clients and monitors
            self.sim.remove_clients(*disconnected_clients, *deactivated_clients)
            self.sim.remove_monitors(*monitors_to_remove)

    def _sequential_update_loop(self) -> None:
        """Sequential simulation update loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Running a sequential simulation update loop.')

        sim_timestep: float = self.sim.mj_model.opt.timestep * self.sim.n_substeps
        cycle_start: float = time.time() - sim_timestep

        # sequential simulation update loop
        while not self._shutdown:
            # filter clients / monitors by state, as their state may change during this simulation step
            # this also simplifies client / monitor removal from the central client / monitor lists
            _, ready_clients, active_clients, disconnected_clients = self.sim.filter_clients()
            active_monitors, monitors_to_remove = self.sim.filter_monitors()

            # handle disconnected clients
            self.sim.deactivate_clients(disconnected_clients)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # Note: Actions need to be collected before sending perceptions to clients in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            client_actions = self.sim.collect_actions(active_clients, block=self.sync_mode)

            # collect monitor commands
            monitor_commands = self.sim.collect_commands(active_monitors)

            # progress simulation
            self.sim.step(client_actions, monitor_commands)

            # handle ready clients
            activated_clients, deactivated_clients = self.sim.activate_clients(ready_clients)
            active_clients.extend(activated_clients)

            # Note: In sequential mode, perceptions should ideally be sent directly after the simulation step to give the agents as much time as possible, while the server notifies monitors, etc.
            self.sim.generate_perceptions(active_clients)
            self.sim.send_perceptions(active_clients)

            # update connected monitors
            self.sim.update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log client perceptions and actions to client logs

            # remove disconnected clients and monitors
            self.sim.remove_clients(*disconnected_clients, *deactivated_clients)
            self.sim.remove_monitors(*monitors_to_remove)
