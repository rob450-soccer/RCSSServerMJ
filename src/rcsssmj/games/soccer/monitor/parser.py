import logging
from random import randint

from rcsssmj.games.soccer.monitor.command import DropBallCommand, KickOffCommand, SetPlayModeCommand, MovePlayerCommand, \
    NoCommand
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.parser import DefaultCommandParser
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class SoccerCommandParser(DefaultCommandParser):
    """Soccer monitor command message parser implementation."""

    def parse_node(self, node: SExpression) -> MonitorCommand | None:
        """Try parsing a soccer monitor command node into a monitor command."""

        n_elements = len(node)

        if node[0] == b'dropBall':
            # drop-ball command: (dropBall)
            return DropBallCommand()

        if node[0] == b'kickOff':
            # kick-off command: (kickOff [Left|Right])
            # random kick-off: (kickOff)
            team_id = randint(0, 1)

            if n_elements > 1 and node[1] == b'Left':
                team_id = 0
            elif n_elements > 1 and node[1] == b'Right':
                team_id = 1

            return KickOffCommand(team_id)

        if node[0] == b'ball':
            # place ball command: (ball (pos <x> <y> <z>) (vel <vx> <vy> <vz>))
            pos: tuple[float, float] | None = None
            for sub_node in node:
                if isinstance(sub_node, SExpression) and sub_node[0] == b'pos':
                    pos = (sub_node.get_float(1), sub_node.get_float(2))
                    break

            return DropBallCommand(pos)

        if node[0] == b'agent':
            # place agent command: (agent (unum <player_number>) (team <team_name>) (pos <x> <y> <z>))
            # alternative command: (agent (unum <player_number>) (team <team_name>) (move <x> <y> <z> <theta>))
            player_id = 0
            team_name = ""
            pos3d: tuple[float, float, float] = (0, 0, 0)
            have_pos = False
            for sub_node in node:
                if isinstance(sub_node, SExpression) and sub_node[0] == b'unum':
                    player_id = sub_node.get_int(1)
                if isinstance(sub_node, SExpression) and sub_node[0] == b'team':
                    team_name = sub_node.get_str(1)
                if isinstance(sub_node, SExpression) and sub_node[0] == b'pos':
                    pos3d = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))
                    have_pos = True

            if have_pos:
                return MovePlayerCommand(player_id, team_name, pos3d)
            else:
                # TODO log error have beam command without position
                return NoCommand("beam agent command without pos node")

        if node[0] == b'playMode':
            # set play mode command: (playMode <play_mode>)
            return SetPlayModeCommand(node.get_str(1))

        return super().parse_node(node)
