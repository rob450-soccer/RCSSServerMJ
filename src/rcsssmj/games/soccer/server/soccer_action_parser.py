import logging
from math import radians

from rcsssmj.games.soccer.sim.soccer_actions import BeamAction
from rcsssmj.games.soccer.sim.soccer_sim_interfaces import PSoccerSimActionInterface
from rcsssmj.server.action_parser import DefaultActionParser
from rcsssmj.sim.actions import SimAction
from rcsssmj.sim.sim_interfaces import PSimActionInterface
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class SoccerActionParser(DefaultActionParser[PSoccerSimActionInterface]):
    """Soccer action message parser implementation."""

    def parse_node(self, node: SExpression, model_prefix: str) -> SimAction[PSimActionInterface] | SimAction[PSoccerSimActionInterface] | None:
        n_elements = len(node)

        if node[0] == b'beam' and n_elements == 4:
            # beam action (beam <x> <y> <theta>)
            return BeamAction(model_prefix + 'beam', (node.get_float(1), node.get_float(2), radians(node.get_float(3))))

        return super().parse_node(node, model_prefix)
