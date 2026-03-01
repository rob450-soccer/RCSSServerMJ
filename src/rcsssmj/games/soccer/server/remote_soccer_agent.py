from rcsssmj.games.soccer.server.soccer_action_parser import SoccerActionParser
from rcsssmj.games.soccer.sim.soccer_agent import SoccerAgent
from rcsssmj.games.soccer.sim.soccer_sim import SoccerSimulation
from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.server.action_parser import ActionParser
from rcsssmj.server.communication.connection import PConnection
from rcsssmj.server.perception_encoder import DefaultPerceptionEncoder, PerceptionEncoder
from rcsssmj.server.remote_agent import TypedRemoteAgent


class RemoteSoccerAgent(TypedRemoteAgent[SoccerSimulation, PSoccerSimActionInterface]):
    """Remote simulation agent representing a soccer player."""

    def __init__(
        self,
        conn: PConnection,
        parser: ActionParser[PSoccerSimActionInterface] | None = None,
        encoder: PerceptionEncoder | None = None,
    ) -> None:
        """Construct a new remote soccer player.

        Parameter
        ---------
        conn: PConnection
            The agent connection.

        parser: ActionParser[PSoccerSimActionInterface] | None, default=None
            The action message parser instance. If ``None`` the default soccer action parser is used.

        encoder: PerceptionEncoder, default=None
            The perception message encoder instance. If ``None`` the default perception encoder is used.
        """

        super().__init__(
            conn,
            SoccerActionParser() if parser is None else parser,
            DefaultPerceptionEncoder() if encoder is None else encoder,
        )

        self._team_name: str = ''
        """The name of the team the agent belongs to."""

        self._player_no: int = -1
        """The player number."""

        self._sim_agent: SoccerAgent | None = None
        """The soccer player simulation agent instance associated with this remote agent."""

    @property
    def team_name(self) -> str:
        """The name of the team this agent belongs to."""

        return self._team_name

    @property
    def player_no(self) -> int:
        """The player number of this agent."""

        return self._player_no

    @property
    def sim_agent(self) -> SoccerAgent | None:
        return self._sim_agent

    def _activate(self, sim: SoccerSimulation) -> None:
        self._sim_agent = sim.add_players([self])[0]

    def deactivate(self, sim: SoccerSimulation) -> None:
        if self._sim_agent is not None:
            sim.remove_players([self._sim_agent])
            self._sim_agent = None

    def _parse_init(self, msg: bytes | bytearray) -> bool:
        init_action = self._parser.parse_init(msg)

        if init_action is None:
            return False

        # set agent information
        self._model_name = init_action.model_name
        self._team_name = init_action.team_name
        self._player_no = init_action.player_no

        return True

    def __str__(self) -> str:
        return f'{self._team_name} #{self._player_no} @ {self._conn}'

    def __repr__(self) -> str:
        return f'RemoteSoccerAgent({self._conn.__repr__()})'
