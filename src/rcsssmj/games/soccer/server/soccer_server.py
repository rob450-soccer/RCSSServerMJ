from typing import TYPE_CHECKING

from rcsssmj.games.soccer.server.remote_soccer_agent import RemoteSoccerAgent
from rcsssmj.games.soccer.server.soccer_action_parser import SoccerActionParser
from rcsssmj.games.soccer.server.soccer_command_parser import SoccerCommandParser
from rcsssmj.games.soccer.sim.soccer_sim import SoccerSimulation
from rcsssmj.server.communication.tcp_connection_listener import TCPConnectionListener
from rcsssmj.server.perception_encoder import DefaultPerceptionEncoder, PerceptionEncoder
from rcsssmj.server.remote_monitor import RemoteMonitor
from rcsssmj.server.server import SimServer

if TYPE_CHECKING:
    from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
    from rcsssmj.server.action_parser import ActionParser
    from rcsssmj.server.command_parser import CommandParser


class SoccerSimServer(SimServer[SoccerSimulation]):
    """The soccer simulation server."""

    def __init__(
        self,
        sim: SoccerSimulation,
        host: str = '127.0.0.1',
        agent_port: int = 60000,
        monitor_port: int = 60001,
        *,
        sequential_mode: bool = False,
        sync_mode: bool = False,
        real_time: bool = True,
        render: bool = True,
    ) -> None:
        """Construct a new soccer simulation server.

        Parameter
        ---------
        sim: SoccerSimulation
            The simulation to run.

        host: str
            The server host address.

        agent_port: int, default=60000
            The port on which to listen for incoming TCP agent connections.

        monitor_port: int, default=60001
            The port on which to listen for incoming TCP monitor connections.

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

        super().__init__(
            sim=sim,
            sequential_mode=sequential_mode,
            sync_mode=sync_mode,
            real_time=real_time,
            render=render,
        )

        self._action_parser: ActionParser[PSoccerSimActionInterface] = SoccerActionParser()
        """Parser for parsing soccer agent actions."""

        self._perception_encoder: PerceptionEncoder = DefaultPerceptionEncoder()
        """Encoder for encoding soccer agent perceptions."""

        self._command_parser: CommandParser = SoccerCommandParser()
        """Parser for parsing soccer monitor commands."""

        # register agent connection listeners
        self._connection_listeners.append(
            TCPConnectionListener(
                lambda conn: self._run_remote_agent(RemoteSoccerAgent(conn, self._action_parser, self._perception_encoder)),
                'agent',
                host,
                agent_port,
            )
        )

        # register monitor connection listeners
        self._connection_listeners.append(
            TCPConnectionListener(
                lambda conn: self._run_remote_monitor(RemoteMonitor(conn, self._command_parser)),
                'monitor',
                host,
                monitor_port,
            )
        )
