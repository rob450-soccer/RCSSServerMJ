import logging
import time
from collections.abc import Sequence
from queue import Empty
from threading import Lock, Thread
from typing import TYPE_CHECKING, Final, Generic, Protocol, TypeVar

from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.server.remote_agent import RemoteAgent, RemoteAgentState
from rcsssmj.server.remote_monitor import RemoteMonitor, RemoteMonitorState, SimMonitor
from rcsssmj.sim.commands import MonitorCommand
from rcsssmj.sim.simulation import BaseSimulation

if TYPE_CHECKING:
    from rcsssmj.server.communication.connection_listener import PConnectionListener

logger = logging.getLogger(__name__)


class PServerComponent(Protocol):
    """Protocol for server components likely running in own threads."""

    def shutdown(self) -> None:
        """Shutdown the component."""

    def join(self) -> None:
        """Wait until the component thread terminates (if existing)."""


S = TypeVar('S', bound=BaseSimulation)


class SimServer(Generic[S]):
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
        sim: S,
        *,
        sequential_mode: bool = False,
        sync_mode: bool = False,
        real_time: bool = True,
        render: bool = True,
    ) -> None:
        """Construct a new simulation sever.

        Parameter
        ---------
        sim: S
            The simulation to run.

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

        self.sim: Final[S] = sim
        """The simulation to run."""

        self.sequential_mode: Final[bool] = sequential_mode
        """Flag for enabling / disabling sequential mode."""

        self.sync_mode: Final[bool] = sync_mode or sequential_mode or not real_time
        """Flag for enabling / disabling sync mode."""

        self.real_time: Final[bool] = real_time
        """Flag for enabling / disabling real-time mode."""

        self.render: Final[bool] = render
        """Flag for enabling / disabling the internal mujoco viewer monitor."""

        self._connection_listeners: Final[list[PConnectionListener]] = []
        """The list of connection listeners."""

        self._agents: Final[list[RemoteAgent[S]]] = []
        """The list of connected agents."""

        self._monitors: Final[list[SimMonitor]] = []
        """The list of connected monitors."""

        self._mutex: Lock = Lock()
        """Mutex for synchronizing simulation threads."""

        self._shutdown: bool = True
        """Flag indicating a shutdown request, causing the simulation server to shutdown."""

    def run(self) -> None:
        """Run simulation server."""

        # 1. SETUP: Setup sockets and start server threads
        logger.info('Starting server...')
        self._shutdown = False

        # setup connection listeners
        try:
            for cl in self._connection_listeners:
                cl.bind()
        except ConnectionError:
            for cl in self._connection_listeners:
                cl.shutdown()
            raise

        # start connection listeners
        for cl in self._connection_listeners:
            cl.listen_for_connections()

        # create simulator threads
        sim_thread = Thread(target=self._run_simulation, name='sim_loop')

        # start simulation server thread
        sim_thread.start()

        logger.info('Starting server... DONE!')

        # 2. RUN: Wait until simulation thread finished
        sim_thread.join()  # run simulation loop in separate thread to isolate exceptions and allow the main thread to clean up

        # 3. CLEANUP: Shutdown everything and wait for socket threads to finish
        logger.info('Shutting down Server...')
        self._shutdown = True

        # shutdown connection listeners
        self._graceful_shutdown(self._connection_listeners)
        self._connection_listeners.clear()

        # shutdown active agents
        self._graceful_shutdown(self._agents)
        self._agents.clear()
        logger.info('Disconnected agents.')

        # shutdown active monitors
        self._graceful_shutdown(self._monitors)
        self._monitors.clear()
        logger.info('Disconnected monitors.')

        # shutdown simulation
        self.sim.shutdown()
        logger.info('Shutdown simulation.')

        logger.info('Shutting down server... DONE!')

    def _graceful_shutdown(self, components: Sequence[PServerComponent]) -> None:
        """Shutdown the given list of components and wait until all component threads have terminated."""

        # trigger shutdown
        for c in components:
            c.shutdown()

        # wait for component threads to finish
        for c in components:
            c.join()

    def shutdown(self) -> None:
        """Request server shutdown."""

        self._shutdown = True

        logger.info('Shutdown requested.')

    def _run_remote_agent(self, agent: RemoteAgent[S]) -> None:
        """Register the given remote agent to the server and run its receive loop.

        Parameter
        ---------
        agent: RemoteAgent[S]
            The agent instance to run.
        """

        with self._mutex:
            self._agents.append(agent)

        # run receive loop
        agent.run()

    def _run_remote_monitor(self, monitor: RemoteMonitor) -> None:
        """Register the given remote monitor to the server and run its receive loop.

        Parameter
        ---------
        monitor: RemoteMonitor
            The monitor instance to run.
        """

        with self._mutex:
            self._monitors.append(monitor)

        # run receive loop
        monitor.run()

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
            self._deactivate_agents(disconnected_agents)

            # handle ready agents
            activated_agents = self._activate_agents(ready_agents)

            # generate perceptions
            self.sim.generate_perceptions()

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # collect agent actions
            # Note: Actions need to be collected before sending perceptions to agents in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            self._collect_actions(active_agents, block=self.sync_mode)

            # send perceptions
            self._send_perceptions(*active_agents, *activated_agents)

            # collect monitor commands
            monitor_commands = self._collect_commands(active_monitors)

            # progress simulation
            self.sim.step(monitor_commands)

            # update connected monitors
            self._update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log agent perceptions and actions to agent logs

            # remove disconnected agents and monitors
            self._remove_agents(disconnected_agents)
            self._remove_monitors(monitors_to_remove)

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
            self._deactivate_agents(disconnected_agents)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start = time.time()

            # Note: Actions need to be collected before sending perceptions to agents in parallel mode to prevent fetching new actions that arrived while still sending perceptions.
            self._collect_actions(active_agents, block=self.sync_mode)

            # collect monitor commands
            monitor_commands = self._collect_commands(active_monitors)

            # progress simulation
            self.sim.step(monitor_commands)

            # handle ready agents
            activated_agents = self._activate_agents(ready_agents)

            # Note: In sequential mode, perceptions should ideally be sent directly after the simulation step to give the agents as much time as possible, while the server notifies monitors, etc.
            self.sim.generate_perceptions()
            self._send_perceptions(*active_agents, *activated_agents)

            # update connected monitors
            self._update_monitors(active_monitors)

            # TODO: log monitor message to simulator log
            # TODO: log agent perceptions and actions to agent logs

            # remove disconnected agents and monitors
            self._remove_agents(disconnected_agents)
            self._remove_monitors(monitors_to_remove)

    def _filter_agents(self) -> tuple[list[RemoteAgent[S]], list[RemoteAgent[S]], list[RemoteAgent[S]], list[RemoteAgent[S]]]:
        """Filter remote agents by state.

        Returns
        -------
        connected_agents: list[RemoteAgent]
            The list of agents in connected state.

        ready_agents: list[RemoteAgent]
            The list of agents in ready state.

        active_agents: list[RemoteAgent]
            The list of agents in active state.

        disconnected_agents: list[RemoteAgent]
            The list of agents in disconnected state.
        """

        connected_agents: list[RemoteAgent[S]] = []
        ready_agents: list[RemoteAgent[S]] = []
        active_agents: list[RemoteAgent[S]] = []
        disconnected_agents: list[RemoteAgent[S]] = []

        with self._mutex:
            for agent in self._agents:
                state = agent.state
                if state == RemoteAgentState.INIT:
                    connected_agents.append(agent)
                elif state == RemoteAgentState.READY:
                    ready_agents.append(agent)
                elif state == RemoteAgentState.ACTIVE:
                    active_agents.append(agent)
                # elif state == SimAgentState.DISCONNECTED:
                else:
                    disconnected_agents.append(agent)

        return connected_agents, ready_agents, active_agents, disconnected_agents

    def _activate_agents(self, agents: Sequence[RemoteAgent[S]]) -> list[RemoteAgent[S]]:
        """Try activate the given list of remote agents.

        Parameter
        ---------
        ready_agents: Sequence[RemoteAgent]
            The list of remote agents to activate.

        Returns
        -------
        activated_agents: list[RemoteAgent[S]]
            The list of successfully activated remote agent instances.
        """

        activated_agents: list[RemoteAgent[S]] = []

        for agent in agents:
            agent.activate(self.sim)

            if agent.state == RemoteAgentState.ACTIVE:
                activated_agents.append(agent)

        return activated_agents

    def _deactivate_agents(self, agents: Sequence[RemoteAgent[S]]) -> None:
        """Deactivate the given list of remote agents.

        Parameter
        ---------
        agents: Sequence[RemoteAgent]
            The list of remote agent instances to deactivate.
        """

        for agent in agents:
            agent.deactivate(self.sim)

    def _send_perceptions(self, *agents: RemoteAgent[S]) -> None:
        """Send the previously generated perceptions to the given agents.

        Parameter
        ---------
        *agents: RemoteAgent
            The remote agent instances to which to send perception information.
        """

        for agent in agents:
            agent.send_perceptions()

    def _collect_actions(
        self,
        agents: Sequence[RemoteAgent[S]],
        *,
        block: bool = False,
        timeout: float = 5,
    ) -> None:
        """Collect the actions from all active agents.

        Parameter
        ---------
        agents: Sequence[SimAgent]
            The list of active agents.

        block: bool, default=False
            Wait for agent actions to arrive.

        timeout: float, default=5
            The time to wait for agent actions to arrive. After this time, the agent is considered inactive and will be shutdown.
            If timeout is a negative number, it will wait forever.
        """

        # collect and buffer all pending agent actions for the next simulation cycle
        for agent in agents:
            agent.collect_actions(block=block, timeout=timeout)

    def _remove_agents(self, agents: Sequence[RemoteAgent[S]]) -> None:
        """Remove the given list of agents from the simulation.

        Note:
        This method will not automatically deactivate the given agents.
        Make sure to deactivate the agent instances before calling this method.

        Parameter
        ---------
        agents: Sequence[SimAgent]
            The agent instances to remove.
        """

        with self._mutex:
            for agent in agents:
                self._agents.remove(agent)

                logger.info('Agent %s removed.', agent)

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
                if monitor.state == RemoteMonitorState.ACTIVE:
                    active_monitors.append(monitor)
                else:
                    inactive_monitors.append(monitor)

        return active_monitors, inactive_monitors

    def _collect_commands(self, monitors: Sequence[SimMonitor]) -> list[MonitorCommand]:
        """Collect the commands from all active monitors.

        Parameter
        ---------
        monitors: Sequence[SimMonitor]
            The list of active monitors.
        """

        monitor_commands: list[MonitorCommand] = []

        for monitor in monitors:
            command_queue = monitor.command_queue
            try:
                while command_queue.qsize() > 0:
                    monitor_commands.append(command_queue.get_nowait())
            except Empty:
                pass

        return monitor_commands

    def _update_monitors(self, monitors: Sequence[SimMonitor]) -> None:
        """Update active monitors.

        Parameter
        ---------
        monitors: Sequence[SimMonitor]
            The list of active monitors.
        """

        state_info = self.sim.generate_state_information()

        for monitor in monitors:
            monitor.update(state_info, self.sim.frame_id)

    def _remove_monitors(self, monitors: Sequence[SimMonitor]) -> None:
        """Remove the given monitors from the simulation.

        Note:
        This method will not automatically deactivate the given monitors.
        Make sure to shutdown the monitor instances before calling this method.

        Parameter
        ---------
        monitors: Sequence[SimMonitor]
            The list of monitor instances to remove.
        """

        with self._mutex:
            for monitor in monitors:
                self._monitors.remove(monitor)

                logger.info('Monitor %s removed.', monitor)
