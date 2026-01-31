import logging
from math import radians
from random import randint

from rcsssmj.games.soccer.sim.soccer_commands import DropBallCommand, KickOffCommand, PlaceBallCommand, PlacePlayerCommand, SetPlayModeCommand
from rcsssmj.server.command_parser import DefaultCommandParser
from rcsssmj.sim.commands import MonitorCommand
from rcsssmj.utils.mjutils import quat_from_axis_angle
from rcsssmj.utils.sexpression import SExpression

logger = logging.getLogger(__name__)


class SoccerCommandParser(DefaultCommandParser):
    """Soccer monitor command message parser implementation."""

    def parse_node(self, node: SExpression) -> MonitorCommand | None:
        """Try parsing a soccer monitor command node into a monitor command."""

        if node[0] == b'dropBall':
            # drop-ball command: (dropBall)
            return DropBallCommand()

        if node[0] == b'kickOff':
            return self._parse_kick_off_command(node)

        if node[0] == b'playMode':
            self._parse_play_mode_command(node)

        if node[0] == b'ball':
            return self._parse_ball_command(node)

        if node[0] == b'agent':
            return self._parse_agent_command(node)

        return super().parse_node(node)

    def _parse_kick_off_command(self, node: SExpression) -> MonitorCommand | None:
        """Parse a kick-off command node.

        Kick-off command format:
        ``(kickOff [Left|left|L|l|Right|right|R|r])``

        Random kick-off:
        ``(kickOff)``
        """

        team_id = randint(0, 1)

        if len(node) > 1:
            if node[1] == b'Left' or node[1] == b'left' or node[1] == b'L' or node[1] == b'l':
                team_id = 0
            elif node[1] == b'Right' or node[1] == b'right' or node[1] == b'R' or node[1] == b'r':
                team_id = 1

        return KickOffCommand(team_id)

    def _parse_play_mode_command(self, node: SExpression) -> MonitorCommand | None:
        """Parse a play-mode command node.

        Play-mode command format:
        ``(playMode <play_mode>)``
        """

        if len(node) < 2:
            logger.debug('Play-mode command without play-mode information!')
            return None

        return SetPlayModeCommand(node.get_str(1))

    def _parse_ball_command(self, node: SExpression) -> MonitorCommand | None:
        """Parse a place-ball command node.

        Place-ball command format:
        ``(ball (pos <x> <y> <z>) [(vel <vx> <vy> <vz>)])``
        """

        pos: tuple[float, float, float] | None = None
        vel: tuple[float, float, float] | None = None

        for sub_node in node.expressions():
            if sub_node[0] == b'pos':
                pos = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))

            if sub_node[0] == b'vel':
                vel = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))

        if pos is None:
            logger.debug('Place-ball command without position node!')
            return None

        return PlaceBallCommand(pos, vel)

    def _parse_agent_command(self, node: SExpression) -> MonitorCommand | None:
        """Parse a place-ball command node.

        Place-agent command format:
        ``(agent (unum <player_number>) (team <team_name>) (pos <x> <y> <z>))``

        alternative:
        ``(agent (unum <player_number>) (team <team_name>) (move <x> <y> <z> <theta>))``

        alternative:
        ``(agent (unum <player_number>) (team <team_name>) (move3d <x> <y> <z> <q0> <q1> <q2> <q3>))``
        """

        player_id = -1
        team_name = ''
        pos: tuple[float, float, float] | None = None
        quat: tuple[float, float, float, float] | None = None

        for sub_node in node.expressions():
            if sub_node[0] == b'unum':
                player_id = sub_node.get_int(1)

            elif sub_node[0] == b'team':
                team_name = sub_node.get_str(1)

            elif sub_node[0] == b'pos':
                pos = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))
                quat = None

            elif sub_node[0] == b'move':
                pos = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))
                theta = radians(sub_node.get_float(4))
                quat = quat_from_axis_angle((0, 0, 1), theta)

            elif sub_node[0] == b'move3d':
                pos = (sub_node.get_float(1), sub_node.get_float(2), sub_node.get_float(3))
                quat = (sub_node.get_float(4), sub_node.get_float(5), sub_node.get_float(6), sub_node.get_float(7))

        if player_id < 0:
            logger.debug('Place-agent command without player number node!')
            return None

        if not team_name:
            logger.debug('Place-agent command without team name node!')
            return None

        if player_id < 0 or not team_name or pos is None:
            logger.debug('Place-agent command without position node!')
            return None

        return PlacePlayerCommand(player_id, team_name, pos, quat)
