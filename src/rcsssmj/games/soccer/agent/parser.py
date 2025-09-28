import logging
from math import radians

from rcsssmj.agent.action import SimAction
from rcsssmj.agent.parser import DefaultActionParser
from rcsssmj.games.soccer.agent.action import BeamAction, SayAction
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class SoccerActionParser(DefaultActionParser):
    """Soccer action message parser implementation."""

    def parse_node(self, node: SExpression, model_prefix: str) -> SimAction | None:
        n_elements = len(node)

        if node[0] == b'beam' and n_elements == 4:
            # beam action (beam <x> <y> <theta>)
            return BeamAction(model_prefix + 'beam', (node.get_float(1), node.get_float(2), radians(node.get_float(3))))

        if node[0] == b'say' and n_elements > 1:
            # say action: (say <message>)
            return SayAction(model_prefix + 'say', ' '.join([node.get_str(i) for i in range(1, n_elements)]))

        return super().parse_node(node, model_prefix)
