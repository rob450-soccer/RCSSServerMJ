import logging
from typing import Any, Final

from rcsssmj.agent import AgentID, PAgent
from rcsssmj.client.parser import SoccerActionParser
from rcsssmj.client.perception import Perception
from rcsssmj.game.referee import SoccerReferee
from rcsssmj.monitor.parser import SoccerCommandParser
from rcsssmj.simulation import BaseSimulation

logger = logging.getLogger(__name__)


class SoccerSimulation(BaseSimulation):
    def __init__(
        self,
        referee: SoccerReferee,
    ) -> None:
        """Construct a new simulation sever.

        Parameter
        ---------
        referee: SoccerReferee
            The soccer referee instance used for judging the match situation.
        """

        super().__init__(
            action_parser=SoccerActionParser(),
            command_parser=SoccerCommandParser(),
            vision_interval=2,
        )

        self.referee: Final[SoccerReferee] = referee
        """The game referee responsible for managing the soccer game aspect of the simulation."""

    def _create_world(self) -> Any | None:
        return self.referee.init_game(self.spec_provider)

    def _request_participation(self, agent: PAgent, model_spec: Any) -> AgentID | None:
        return self.referee.request_participation(agent, model_spec)

    def _handle_withdrawal(self, agent_id: AgentID) -> None:
        return self.referee.handle_withdrawal(agent_id)

    def _generate_game_state_perception(self) -> Perception:
        return self.referee.generate_perception()
