from abc import ABC, abstractmethod

from rcsssmj.client.action import BeamAction, InitRequest, MotorAction, SayAction, SimAction


class ActionParser(ABC):
    """
    Base class for action message parsers.
    """

    @abstractmethod
    def parse_init(self, data: bytes | bytearray) -> InitRequest | None:
        """
        Parse an initialization message.
        """

    @abstractmethod
    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """
        Parse an action message.
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

        msg = data.decode()

        if msg[0:5] != '(init' or msg[-1] != ')':
            return None

        splits = msg[6:-1].split(' ')

        if len(splits) != 3:
            return None

        # set client specific attributes
        model_name: str = splits[0]
        team_name: str = splits[1]
        player_no: int = abs(int(splits[2])) % 100

        return InitRequest(model_name, team_name, player_no)

    def parse_action(self, data: bytes | bytearray, model_prefix: str) -> list[SimAction]:
        """
        Try parsing an action message containing arbitrary actuators.
        """

        actions: list[SimAction] = []

        # parse individual actions from message
        idx = 0
        try:
            while idx < len(data):
                chunks, idx = self._parse_node(data, idx)

                if chunks[0] == b'beam':
                    # beam action (beam <x> <y> <theta>)
                    if len(chunks) == 4:
                        actions.append(BeamAction(model_prefix + 'beam', (float(chunks[1]), float(chunks[2]), float(chunks[3]))))

                elif chunks[0] == b'say':
                    # say action: (say <message>)
                    if len(chunks) > 1:
                        actions.append(SayAction(model_prefix + 'say', ' '.join([chunk.decode() for chunk in chunks[1:]])))

                elif chunks[0] == b'syn':
                    # sync action: (syn)
                    pass

                elif len(chunks) == 2:
                    # joint action: (<name> <velocity>)
                    actions.append(MotorAction(model_prefix + chunks[0].decode(), float(chunks[1])))

        except RuntimeError:
            # error while parsing
            pass

        return actions

    def _parse_node(self, data: bytes | bytearray, start: int) -> tuple[list[bytes | bytearray], int]:
        """
        Try parsing an expression node.
        """

        if data[start] != ord('('):
            raise RuntimeError

        chunks: list[bytes | bytearray] = []
        start_idx: int = start + 1
        idx: int = start_idx

        while idx < len(data):
            if data[idx] == ord(' '):
                if idx > start_idx:
                    chunks.append(data[start_idx:idx])
                start_idx = idx + 1
            if data[idx] == ord(')'):
                if idx > start_idx:
                    chunks.append(data[start_idx:idx])
                return chunks, idx + 1
            if data[idx] == ord('('):
                raise RuntimeError
            idx += 1

        if idx > start_idx:
            chunks.append(data[start_idx:idx])

        return chunks, idx
