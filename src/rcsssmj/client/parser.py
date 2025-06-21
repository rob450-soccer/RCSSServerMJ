import logging
from abc import ABC, abstractmethod

from rcsssmj.client.action import BeamAction, InitRequest, MotorAction, SayAction, SimAction
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class ActionParser(ABC):
    """Base class for client action message parsers."""

    @abstractmethod
    def parse_init(self, data: bytes | bytearray) -> InitRequest | None:
        """Parse a client initialization message."""

    @abstractmethod
    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """Parse a client action message."""


class DefaultActionParser(ActionParser):
    """Default action message parser implementation based on symbolic expressions."""

    def parse_init(self, data: bytes | bytearray) -> InitRequest | None:
        """
        Try parsing an initialization message in the form:

        (init <robot_model> <team_name> <player_no>)
        """

        try:
            node: SExpression = SExpression.from_array(data).get_expr(0)

            if node[0] != b'init' or len(node) != 4:
                return None

            model_name: str = node.get_str(1)
            team_name: str = node.get_str(2)
            player_no: int = abs(node.get_int(3)) % 100
        except Exception:  # noqa: BLE001
            return None

        return InitRequest(model_name, team_name, player_no)

    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """Try parsing an action message into individual simulation client actions."""

        actions: list[SimAction] = []

        # parse individual actions from message
        try:
            node: SExpression = SExpression.from_array(data)

            for child in node.expressions():
                action = self.parse_node(child, model_prefix)
                if action is not None:
                    actions.append(action)

        except Exception:  # noqa: BLE001
            # error while parsing
            logger.debug('Error parsing action message for model: %s.', model_prefix, exc_info=True)

        return actions

    def parse_node(self, node: SExpression, model_prefix: str) -> SimAction | None:
        """Try parsing an action message node into an simulation client action."""

        n_elements = len(node)

        if node[0] == b'syn':
            # sync action: (syn)
            return None

        if n_elements == 6:
            # joint action: (<name> <q> <dq> <kp> <kd> <tau>)
            return MotorAction(
                model_prefix + node.get_str(0),
                node.get_float(1),
                node.get_float(2),
                node.get_float(3),
                node.get_float(4),
                node.get_float(5),
            )

        logger.debug('Unknown action node: %s', node)

        return None


class SoccerActionParser(DefaultActionParser):
    """Soccer action message parser implementation."""

    def parse_node(self, node: SExpression, model_prefix: str) -> SimAction | None:
        n_elements = len(node)

        if node[0] == b'beam' and n_elements == 4:
            # beam action (beam <x> <y> <theta>)
            return BeamAction(model_prefix + 'beam', (node.get_float(1), node.get_float(2), node.get_float(3)))

        if node[0] == b'say' and n_elements > 1:
            # say action: (say <message>)
            return SayAction(model_prefix + 'say', ' '.join([node.get_str(i) for i in range(1, n_elements)]))

        return super().parse_node(node, model_prefix)
