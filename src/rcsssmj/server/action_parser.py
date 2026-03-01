import base64
import binascii
import logging
from abc import ABC, abstractmethod
from math import radians
from typing import Generic, TypeVar

from rcsssmj.sim.actions import InitRequest, MotorAction, SimAction, SpeakerAction
from rcsssmj.sim.sim_interfaces import PSimActionInterface
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


SAI = TypeVar('SAI', bound=PSimActionInterface)


class ActionParser(ABC, Generic[SAI]):
    """Base class for simulation agent action message parsers."""

    @abstractmethod
    def parse_init(self, data: bytes | bytearray) -> InitRequest | None:
        """Parse a agent initialization message."""

    @abstractmethod
    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction[PSimActionInterface] | SimAction[SAI]]:
        """Parse a agent action message."""


class DefaultActionParser(ActionParser[SAI]):
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

    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction[PSimActionInterface] | SimAction[SAI]]:
        """Try parsing an action message into individual simulation agent actions."""

        actions: list[SimAction[PSimActionInterface] | SimAction[SAI]] = []

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

    def parse_node(self, node: SExpression, model_prefix: str) -> SimAction[PSimActionInterface] | SimAction[SAI] | None:
        """Try parsing an action message node into an simulation agent action."""

        n_elements = len(node)

        if node[0] == b'syn':
            # sync action: (syn)
            return None

        if node[0] == b'SPK' and n_elements > 3:
            # say action: (SPK <name> <volume> <message>)
            try:
                return SpeakerAction(
                    model_prefix + node.get_str(1),
                    node.get_float(2) / 100,
                    base64.b64decode(node.get_str(3), validate=True),
                )
            except binascii.Error:
                return None

        if n_elements == 6:
            # joint action: (<name> <q> <dq> <kp> <kd> <tau>)
            return MotorAction(
                model_prefix + node.get_str(0),
                radians(node.get_float(1)),
                radians(node.get_float(2)),
                node.get_float(3),
                node.get_float(4),
                node.get_float(5),
            )

        logger.debug('Unknown action node: %s', node)

        return None
