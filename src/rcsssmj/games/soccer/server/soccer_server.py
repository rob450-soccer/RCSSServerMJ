from rcsssmj.games.soccer.server.soccer_action_parser import SoccerActionParser
from rcsssmj.games.soccer.server.soccer_command_parser import SoccerCommandParser
from rcsssmj.games.soccer.sim.soccer_sim import SoccerSimulation
from rcsssmj.server.server import SimServer


class SoccerSimServer(SimServer):
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
        """Construct a new soccer simulation server."""

        super().__init__(
            sim,
            host,
            agent_port,
            monitor_port,
            sequential_mode=sequential_mode,
            sync_mode=sync_mode,
            real_time=real_time,
            render=render,
            action_parser=SoccerActionParser(),
            perception_encoder=None,
            command_parser=SoccerCommandParser(),
        )
