import logging
import socket
import time
from threading import Lock, Thread
from typing import Final

from rcsssmj.agent.encoder import DefaultPerceptionEncoder, PerceptionEncoder
from rcsssmj.agent.parser import ActionParser, DefaultActionParser
from rcsssmj.agent.sim_agent import RemoteSimAgent, SimAgent, SimAgentState
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.monitor.parser import CommandParser, DefaultCommandParser
from rcsssmj.monitor.sim_monitor import RemoteSimMonitor, SimMonitor, SimMonitorState
from rcsssmj.simulation import BaseSimulation

logger = logging.getLogger(__name__)


class SimServer:
    """The simulation server component.

    The simulation server is the core server component, responsible for running the central simulation loop as well as managing agent and monitor connections / communication.
    Game specific logic is encapsulated in a referee instance, which is incorporated into the simulation loop.

    By default, the simulation server runs in a competition setup mode.
    This means that it will try to simulate in real time and will not wait for agent actions to arrive before the next simulation cycle.
    In this scenario, connected agents are responsible for managing their resources and performance to respond in time (as it is the case for a real robot, too).

    However, the server also offers a set of flags with which you can setup the server in a training / evaluation mode.
    Use the `sync_mode` flag to tell the server to wait for an action response of all active agents before simulating the next simulation cycle.
    When disabling the `real_time` flag, the server will not wait between simulation cycles to simulate a real-time scenario.
    Instead, it will directly progress to simulating the next simulation cycle - aka run "as-fast-as-possible".
    When disabling the `real_time` flag, it is advised to activate sync mode, too, as otherwise there will be some severe desync between server and agent processes.

    At this point in time, the simulation server comes with a built-in mujoco viewer as internal monitor, which will be started by default.
    Note that due to the Python GIL and the fact that the mujoco viewer is designed to run synchronously, rendering will impact the real-time capability of the simulation in certain scenarios.
    You can disable the internal monitor component by setting the `render` flag to `False`.
    """

    def __init__(
        self,
        sim: BaseSimulation,
        host: str = '127.0.0.1',
        agent_port: int = 60000,
        monitor_port: int = 60001,
        *,
        sequential_mode: bool = False,
        sync_mode: bool = False,
        real_time: bool = True,
        render: bool = True,
        action_parser: ActionParser | None = None,
        perception_encoder: PerceptionEncoder | None = None,
        command_parser: CommandParser | None = None,
    ) -> None:
        """Construct a new simulation sever.

        Parameter
        ---------
        host: str
            The server host address.

        agent_port: int, default=60000
            The port on which to listen for incoming agent connections.

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

        action_parser: ActionParser | None, default=None
            Parser instance for parsing remote agent actions.

        perception_encoder: PerceptionEncoder | None, default=None
            Encoder instance for encoding perceptions for remote agents.

        command_parser: CommandParser | None, default=None
            Parser instance for parsing remote monitor commands.
        """

        self.sim: Final[BaseSimulation] = sim
        """The simulation to run."""

        self.host: Final[str] = host
        """The server host address."""

        self.agent_port: Final[int] = agent_port
        """The port on which to listen for incoming agent connections."""

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

        self.action_parser: Final[ActionParser] = DefaultActionParser() if action_parser is None else action_parser
        """Parser for parsing agent action messages."""

        self.perception_encoder: Final[PerceptionEncoder] = DefaultPerceptionEncoder() if perception_encoder is None else perception_encoder
        """Encoder for encoding agent perception messages."""

        self.command_parser: Final[CommandParser] = DefaultCommandParser() if command_parser is None else command_parser
        """Parser for parsing monitor command messages."""

        self._agent_sock: socket.socket | None = None
        """The socket for listening for incoming agent connections (only present after the server has been started)."""

        self._monitor_sock: socket.socket | None = None
        """The socket for listening for incoming monitor connections (only present after the server has been started)."""

        self._agents: list[SimAgent] = []
        """The list of connected agents."""

        self._monitors: list[SimMonitor] = []
        """The list of connected monitors."""

        self._mutex: Lock = Lock()
        """Mutex for synchronizing simulation threads."""

        self._shutdown: bool = True
        """Flag indicating a shutdown request, causing the simulation server to shutdown."""

    def run(self) -> None:
        """Run simulation server."""

        if self._agent_sock is not None or self._monitor_sock is not None:
            # a simulation is already running...
            raise RuntimeError

        # 1. SETUP: Setup sockets and start server threads
        logger.info('Starting server...')
        self._shutdown = False

        # setup agent socket
        self._agent_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._agent_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._agent_sock.bind((self.host, self.agent_port))
        self._agent_sock.listen(5)

        # setup monitor socket
        try:
            self._monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._monitor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._monitor_sock.bind((self.host, self.monitor_port))
            self._monitor_sock.listen(5)
        except ConnectionError:
            self._agent_sock.shutdown(socket.SHUT_RDWR)
            self._agent_sock.close()
            raise

        # create simulator threads
        sim_thread = Thread(target=self._run_simulation, name='sim_loop')
        agent_listener_thread = Thread(target=self._listen_for_agents, name='agent_connections_listener')
        monitor_listener_thread = Thread(target=self._listen_for_monitors, name='monitor_connections_listener')

        # start simulator threads
        agent_listener_thread.start()
        monitor_listener_thread.start()
        sim_thread.start()

        logger.info('Starting server... DONE!')

        # 2. RUN: Wait until simulation thread finished
        sim_thread.join()  # run simulation loop in separate thread to isolate exceptions and allow the main thread to clean up

        # 3. CLEANUP: Shutdown everything and wait for socket threads to finish
        logger.info('Shutting down Server...')
        self._shutdown = True

        # shutdown agent and monitor sockets
        try:
            self._agent_sock.shutdown(socket.SHUT_RDWR)
        except Exception:  # noqa: BLE001
            logger.debug('ERROR while shutting down agent socket!', exc_info=True)

        try:
            self._monitor_sock.shutdown(socket.SHUT_RDWR)
        except Exception:  # noqa: BLE001
            logger.debug('ERROR while shutting down monitor socket!', exc_info=True)

        # wait for agent and monitor connection listener threads to finish
        agent_listener_thread.join()
        monitor_listener_thread.join()

        # shutdown active agents
        for agent in self._agents:
            agent.shutdown(wait=True)
        self._agents.clear()
        logger.info('Disconnected agents.')

        # shutdown active monitors
        for monitor in self._monitors:
            monitor.shutdown(wait=True)
        self._monitors.clear()
        logger.info('Disconnected monitors.')

        # shutdown simulation
        self.sim.shutdown()
        logger.info('Shutdown simulation...')

        # cleanup socket refs
        self._agent_sock = None
        self._monitor_sock = None

        logger.info('Shutting down server... DONE!')

    def shutdown(self) -> None:
        """Request server shutdown."""

        self._shutdown = True

        logger.info('Shutdown requested.')

    def _listen_for_agents(self) -> None:
        """Wait for incoming agent connections.

        Note: This method is executed by the agent listener thread - don't call it independently!
        """

        if self._agent_sock is None:
            return

        logger.info('Listening for agent connections on %s:%d', self.host, self.agent_port)
        while not self._shutdown:
            try:
                sock, addr = self._agent_sock.accept()
            except Exception:  # noqa: BLE001
                self._shutdown = True
                break

            logger.info('New agent connection: %s.', addr)

            # create remote agent instances
            conn = TCPLPMConnection(sock, addr)
            agent = RemoteSimAgent(conn, self.action_parser, self.perception_encoder)

            with self._mutex:
                self._agents.append(agent)

        logger.info('Shutdown agent listener thread.')
        self._agent_sock.close()

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

            # create remote monitor instances
            conn = TCPLPMConnection(sock, addr)
            monitor = RemoteSimMonitor(conn, self.command_parser)

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
        if not self.sim.init():
            return

        # create internal monitor
        if self.render:
            with self._mutex:
                self._monitors.append(MujocoMonitor(self.sim.mj_model, 2))

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

        sim_timestep: float = self.sim.timestep
        cycle_start: float = time.time() - sim_timestep

        # parallel simulation update loop
        while not self._shutdown:
            # filter agents / monitors by state, as their state may change during this simulation step
            # this also simplifies agent / monitor removal from the central agent / monitor lists
            _, ready_agents, active_agents, disconnected_agents = self._filter_agents()
            active_monitors, monitors_to_remove = self._filter_monitors()

            # handle disconnected agents
            self.sim.deactivate_agents(disconnected_agents)

            # handle ready agents
            activated_agents, deactivated_agents = self.sim.activate_agents(ready_agents)

            # generate perceptions
            self.sim.generate_perceptions(active_agents + activated_agents)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # collect agent actions
            # Note: Actions need to be collected before sending perceptions to agents in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            agent_actions = self.sim.collect_actions(active_agents, block=self.sync_mode)

            # send perceptions
            self.sim.send_perceptions(active_agents + activated_agents)

            # collect monitor commands
            monitor_commands = self.sim.collect_commands(active_monitors)

            # progress simulation
            self.sim.step(agent_actions, monitor_commands)

            # update connected monitors
            self.sim.update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log agent perceptions and actions to agent logs

            # remove disconnected agents and monitors
            self._remove_agents(*disconnected_agents, *deactivated_agents)
            self._remove_monitors(*monitors_to_remove)

    def _sequential_update_loop(self) -> None:
        """Sequential simulation update loop.

        Note: This method is executed by the simulation thread - don't call it independently!
        """

        logger.info('Running a sequential simulation update loop.')

        sim_timestep: float = self.sim.timestep
        cycle_start: float = time.time() - sim_timestep

        # sequential simulation update loop
        while not self._shutdown:
            # filter agents / monitors by state, as their state may change during this simulation step
            # this also simplifies agent / monitor removal from the central agent / monitor lists
            _, ready_agents, active_agents, disconnected_agents = self._filter_agents()
            active_monitors, monitors_to_remove = self._filter_monitors()

            # handle disconnected agents
            self.sim.deactivate_agents(disconnected_agents)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # Note: Actions need to be collected before sending perceptions to agents in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            agent_actions = self.sim.collect_actions(active_agents, block=self.sync_mode)

            # collect monitor commands
            monitor_commands = self.sim.collect_commands(active_monitors)

            # progress simulation
            self.sim.step(agent_actions, monitor_commands)

            # handle ready agents
            activated_agents, deactivated_agents = self.sim.activate_agents(ready_agents)
            active_agents.extend(activated_agents)

            # Note: In sequential mode, perceptions should ideally be sent directly after the simulation step to give the agents as much time as possible, while the server notifies monitors, etc.
            self.sim.generate_perceptions(active_agents)
            self.sim.send_perceptions(active_agents)

            # update connected monitors
            self.sim.update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log agent perceptions and actions to agent logs

            # remove disconnected agents and monitors
            self._remove_agents(*disconnected_agents, *deactivated_agents)
            self._remove_monitors(*monitors_to_remove)

    def _remove_agents(self, *agents: SimAgent) -> None:
        """Remove the given agents from the simulation.

        Note:
        This method will not automatically deactivate the given agents.
        Make sure to deactivate the agent instances before calling this method.

        Parameter
        ---------
        *agents: SimAgent
            The agent instances to remove.
        """

        with self._mutex:
            for agent in agents:
                self._agents.remove(agent)

    def _remove_monitors(self, *monitors: SimMonitor) -> None:
        """Remove the given monitors from the simulation.

        Note:
        This method will not automatically deactivate the given monitors.
        Make sure to shutdown the monitor instances before calling this method.

        Parameter
        ---------
        *monitors: SimMonitor
            The monitor instances to remove.
        """

        with self._mutex:
            for monitor in monitors:
                self._monitors.remove(monitor)

    def _filter_agents(self) -> tuple[list[SimAgent], list[SimAgent], list[SimAgent], list[SimAgent]]:
        """Filter simulation agents by state.

        Returns
        -------
        connected_agents: list[SimAgent]
            The list of agents in connected state.

        ready_agents: list[SimAgent]
            The list of agents in ready state.

        active_agents: list[SimAgent]
            The list of agents in active state.

        disconnected_agents: list[SimAgent]
            The list of agents in disconnected state.
        """

        connected_agents: list[SimAgent] = []
        ready_agents: list[SimAgent] = []
        active_agents: list[SimAgent] = []
        disconnected_agents: list[SimAgent] = []

        with self._mutex:
            for agent in self._agents:
                state = agent.get_state()
                if state == SimAgentState.INIT:
                    connected_agents.append(agent)
                elif state == SimAgentState.READY:
                    ready_agents.append(agent)
                elif state == SimAgentState.ACTIVE:
                    active_agents.append(agent)
                # elif state == SimAgentState.DISCONNECTED:
                else:
                    disconnected_agents.append(agent)

        return connected_agents, ready_agents, active_agents, disconnected_agents

    def _filter_monitors(self) -> tuple[list[SimMonitor], list[SimMonitor]]:
        """Filter simulation monitors by state.

        Returns
        -------
        active_monitors: list[SimMonitor]
            The list of active / connected monitors.

        inactive_monitors: list[SimMonitor]
            The list of inactive / disconnected monitors.
        """

        active_monitors: list[SimMonitor] = []
        inactive_monitors: list[SimMonitor] = []

        with self._mutex:
            for monitor in self._monitors:
                if monitor.get_state() == SimMonitorState.ACTIVE:
                    active_monitors.append(monitor)
                else:
                    inactive_monitors.append(monitor)

        return active_monitors, inactive_monitors
