from abc import ABC, abstractmethod

from rcsssmj.client.action import BeamAction, InitRequest, MotorAction, SayAction, SimAction
from rcsssmj.utils.sexpression import SExpression


class ActionParser(ABC):
    """
    Base class for client action message parsers.
    """

    @abstractmethod
    def parse_init(self, data: bytes | bytearray) -> InitRequest | None:
        """
        Parse a client initialization message.
        """

    @abstractmethod
    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """
        Parse a client action message.
        """


class SExprActionParser(ActionParser):
    """
    Default symbolic expression action message parser implementation.
    """

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
        except Exception:
            return None

        return InitRequest(model_name, team_name, player_no)

    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """
        Try parsing an action message containing arbitrary actuators.
        """

        actions: list[SimAction] = []

        # parse individual actions from message
        try:
            node: SExpression = SExpression.from_array(data)

            for child in node:
                if not isinstance(child, SExpression):
                    continue

                n_elements = len(child)

                if child[0] == b'beam':
                    # beam action (beam <x> <y> <theta>)
                    if n_elements == 4:
                        actions.append(BeamAction(model_prefix + 'beam', (child.get_float(1), child.get_float(2), child.get_float(3))))

                elif child[0] == b'say':
                    # say action: (say <message>)
                    if n_elements > 1:
                        actions.append(SayAction(model_prefix + 'say', ' '.join([child.get_str(i) for i in range(1, n_elements)])))

                elif child[0] == b'syn':
                    # sync action: (syn)
                    pass

                elif n_elements == 2:
                    # joint action: (<name> <velocity>)
                    actions.append(MotorAction(model_prefix + child.get_str(0), child.get_float(1)))

        except Exception:
            # error while parsing
            pass

        return actions
