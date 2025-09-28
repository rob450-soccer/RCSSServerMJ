import contextlib
import logging
from collections.abc import Mapping, Sequence
from math import degrees, pi
from typing import Any, Final

import mujoco

from rcsssmj.agent.perception import Perception
from rcsssmj.agents import AgentID, PAgent
from rcsssmj.games.soccer.agent.perception import GameStatePerception
from rcsssmj.games.soccer.field import SoccerField
from rcsssmj.games.soccer.game_object import SoccerBall, SoccerPlayer
from rcsssmj.games.soccer.game_state import GameState
from rcsssmj.games.soccer.monitor.state import SoccerEnvironmentInformation, SoccerGameInformation
from rcsssmj.games.soccer.referee import SoccerReferee
from rcsssmj.games.soccer.rules import FIFASoccerRules, SoccerRules
from rcsssmj.games.teams import TeamSide
from rcsssmj.mjutils import place_robot_3d, quat_from_axis_angle
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.state import SimStateInformation
from rcsssmj.simulation import BaseSimulation

logger = logging.getLogger(__name__)


class SoccerSimulation(BaseSimulation):
    """Soccer simulation implementation."""

    def __init__(
        self,
        field: SoccerField,
        rules: SoccerRules | None = None,
        referee: SoccerReferee | None = None,
    ) -> None:
        """Construct a new simulation sever.

        Parameter
        ---------
        field: SoccerField
            The soccer field specification.

        rules: SoccerRules | None, default=None
            The soccer rule set to apply by the soccer referee.

        referee: SoccerReferee | None, default=None
            The soccer referee managing the game aspect of the simulation.
        """

        super().__init__(vision_interval=2)

        self.field: Final[SoccerField] = field
        """The soccer field specification."""

        self.rules: Final[SoccerRules] = FIFASoccerRules() if rules is None else rules
        """The soccer game rule book."""

        self.referee: Final[SoccerReferee] = SoccerReferee() if referee is None else referee
        """The game referee responsible for managing the soccer game aspect of the simulation."""

        self.game_state: Final[GameState] = GameState()
        """The current soccer game state."""

        self.ball: Final[SoccerBall] = SoccerBall()
        """The soccer ball representation."""

        self._team_players: Final[Mapping[int, dict[int, SoccerPlayer]]] = {
            TeamSide.LEFT.value: {},
            TeamSide.RIGHT.value: {},
        }
        """The list of active team players."""

        # set the game instance of the referee to this simulation
        self.referee.game = self

    @property
    def left_players(self) -> Mapping[int, SoccerPlayer]:
        """The active soccer player representations of the left team."""

        return self._team_players[TeamSide.LEFT.value]

    @property
    def right_players(self) -> Mapping[int, SoccerPlayer]:
        """The active soccer player representations of the right team."""

        return self._team_players[TeamSide.RIGHT.value]

    def get_players(self, side: TeamSide | int) -> Mapping[int, SoccerPlayer]:
        """Return the active soccer player representations for the team corresponding to the given side.

        Parameter
        ---------
        side: TeamSide | int
            The team side or side id for which to return the players.
        """

        side = side.value if isinstance(side, TeamSide) else side

        return self._team_players[side]

    def get_all_players(self) -> Sequence[SoccerPlayer]:
        """Return all active soccer player representations (from left and right team)."""

        left_players = list(self._team_players[TeamSide.LEFT.value].values())
        right_players = list(self._team_players[TeamSide.RIGHT.value].values())

        return left_players + right_players

    def init(self) -> bool:
        # forward init call to parent
        if not super().init():
            return False

        # reset soccer simulation states
        self.game_state.reset()
        self._team_players[TeamSide.LEFT.value].clear()
        self._team_players[TeamSide.RIGHT.value].clear()

        # initialize ball
        self.ball.activate(self._mj_spec.body('ball'))

        # initialize referee
        self.referee.reset()

        return True

    def _create_world(self) -> Any | None:
        # load world specification
        world_spec = self.spec_provider.load_environment('soccer')
        if world_spec is None:
            return None

        # manipulate world to match field description
        def add_marker(name: str, x: float, y: float, z: float) -> Any:
            """Helper function for adding markers to the world."""
            site = world_spec.worldbody.add_site()
            site.name = name + '-vismarker'
            site.pos[0] = x
            site.pos[1] = y
            site.pos[2] = z

        field_half_x = self.field.field_dim[0] / 2
        field_half_y = self.field.field_dim[1] / 2
        field_half_z = self.field.field_dim[2] / 2

        goal_half_y = self.field.goal_dim[1] / 2
        goal_z = self.field.goal_dim[2]

        goalie_area_x = field_half_x - self.field.goalie_area_dim[0]
        goalie_area_half_y = self.field.goalie_area_dim[1]

        # resize field
        pitch_geom = world_spec.geom('pitch')
        pitch_geom.size[0] = field_half_x
        pitch_geom.size[1] = field_half_y
        pitch_geom.size[2] = field_half_z

        # add floor around the field
        floor_color = [0.2, 0.5, 0.2, 1]  # dark green color
        left_right_floor_x_size = max(1, field_half_x * 0.05)
        top_bottom_floor_y_size = max(1, field_half_y * 0.05)
        left_floor_body = world_spec.worldbody.add_body(name='left-floor', pos=(-field_half_x, 0, 0))
        left_floor_body.add_geom(
            name='left-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 - left_right_floor_x_size, 0, 0),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        right_floor_body = world_spec.worldbody.add_body(name='right-floor', pos=(field_half_x, 0, 0))
        right_floor_body.add_geom(
            name='right-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0 + left_right_floor_x_size, 0, 0),
            size=(left_right_floor_x_size, field_half_y, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        top_floor_body = world_spec.worldbody.add_body(name='top-floor', pos=(0, field_half_y, 0))
        top_floor_body.add_geom(
            name='top-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 + top_bottom_floor_y_size, 0),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )
        bottom_floor_body = world_spec.worldbody.add_body(name='bottom-floor', pos=(0, -field_half_y, 0))
        bottom_floor_body.add_geom(
            name='bottom-floor',
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=(0, 0 - top_bottom_floor_y_size, 0),
            size=(field_half_x + (left_right_floor_x_size * 2), top_bottom_floor_y_size, field_half_z),
            rgba=floor_color,
            contype=0,
            conaffinity=0,
            group=2,
            density=0,
        )

        # add goals
        goal_post_color = [0.8, 0.8, 0.8, 1]
        goal_net_color = [1, 1, 1, 0.2]
        for side, x_sign in [('left', -1), ('right', 1)]:
            goal_name = f'goal-{side}'
            goal_body = world_spec.worldbody.add_body(
                name=goal_name,
                pos=(x_sign * field_half_x, 0, 0),
            )
            depth = x_sign * self.field.goal_dim[0]

            # vertical posts
            goal_body.add_geom(
                name=f'{goal_name}-front-left-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, -goal_half_y, goal_z / 2],
                size=[self.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-front-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, goal_half_y, goal_z / 2],
                size=[self.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-left-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, -goal_half_y, goal_z / 2],
                size=[self.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-right-post',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, goal_half_y, goal_z / 2],
                size=[self.field.goal_post_radius, goal_z / 2, 0],
                rgba=goal_post_color,
            )

            # crossbars
            goal_body.add_geom(
                name=f'{goal_name}-front-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0, 0, goal_z],
                size=[self.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, 0, goal_z],
                size=[self.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            # side crossbars (roof)
            goal_body.add_geom(
                name=f'{goal_name}-left-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, -goal_half_y, goal_z],
                size=[self.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, goal_half_y, goal_z],
                size=[self.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, -0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )

            # lower crossbars (ground level)
            goal_body.add_geom(
                name=f'{goal_name}-back-lower-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth, 0, 0],
                size=[self.field.goal_post_radius, goal_half_y, 0],
                quat=[0, 0, 0.7071068, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-lower-left-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, -goal_half_y, 0],
                size=[self.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, 0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-lower-right-side-crossbar',
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[depth / 2, goal_half_y, 0],
                size=[self.field.goal_post_radius, abs(depth) / 2, 0],
                quat=[0, -0.7071068 * x_sign, 0, 0.7071068],
                rgba=goal_post_color,
            )

            # nets
            goal_body.add_geom(
                name=f'{goal_name}-top-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, 0, goal_z],
                size=[abs(depth) / 2, goal_half_y, self.field.goal_post_radius],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-back-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth, 0, goal_z / 2],
                size=[self.field.goal_post_radius, goal_half_y, goal_z / 2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-left-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, -goal_half_y, goal_z / 2],
                size=[abs(depth) / 2, self.field.goal_post_radius, goal_z / 2],
                rgba=goal_net_color,
            )
            goal_body.add_geom(
                name=f'{goal_name}-right-side-net',
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[depth / 2, goal_half_y, goal_z / 2],
                size=[abs(depth) / 2, self.field.goal_post_radius, goal_z / 2],
                rgba=goal_net_color,
            )

        # fmt: off
        # field markers
        add_marker('l_luf', -field_half_x,  field_half_y, 0)  # L-junction: left upper field
        add_marker('l_llf', -field_half_x, -field_half_y, 0)  # L-junction: left lower field
        add_marker('l_ruf',  field_half_x,  field_half_y, 0)  # L-junction: right upper field
        add_marker('l_rlf',  field_half_x, -field_half_y, 0)  # L-junction: right lower field

        add_marker('t_cuf', 0,  field_half_y, 0)  # T-junction: center upper field
        add_marker('t_clf', 0, -field_half_y, 0)  # T-junction: center lower field

        # center circle markers
        add_marker('x_cuc', 0,  self.field.center_circle_radius, 0)  # X-junction: center upper circle
        add_marker('x_clc', 0, -self.field.center_circle_radius, 0)  # X-junction: center lower circle

        # penalty spot markers
        penalty_marker_x = field_half_x - self.field.penalty_spot_distance
        add_marker('p_lpm', -penalty_marker_x, 0, 0)  # X-junction: center upper circle
        add_marker('p_rpm',  penalty_marker_x, 0, 0)  # X-junction: center lower circle

        # add goal post markers
        add_marker('g_lup', -field_half_x,  goal_half_y, goal_z)  # Goal: left upper post
        add_marker('g_llp', -field_half_x, -goal_half_y, goal_z)  # Goal: left lower post
        add_marker('g_rup',  field_half_x,  goal_half_y, goal_z)  # Goal: right upper post
        add_marker('g_rlp',  field_half_x, -goal_half_y, goal_z)  # Goal: right lower post

        # goalie area markers
        add_marker('l_luga', -goalie_area_x,  goalie_area_half_y, 0)  # L-junction: left upper goalie area
        add_marker('l_llga', -goalie_area_x, -goalie_area_half_y, 0)  # L-junction: left lower goalie area
        add_marker('l_ruga',  goalie_area_x,  goalie_area_half_y, 0)  # L-junction: right upper goalie area
        add_marker('l_rlga',  goalie_area_x, -goalie_area_half_y, 0)  # L-junction: right lower goalie area

        add_marker('t_luga', -field_half_x,  goalie_area_half_y, 0)  # T-junction: left upper goalie area
        add_marker('t_llga', -field_half_x, -goalie_area_half_y, 0)  # T-junction: left lower goalie area
        add_marker('t_ruga',  field_half_x,  goalie_area_half_y, 0)  # T-junction: right upper goalie area
        add_marker('t_rlga',  field_half_x, -goalie_area_half_y, 0)  # T-junction: right lower goalie area

        # penalty area markers
        if self.field.penalty_area_dim is not None:
            pen_area_x = field_half_x - self.field.penalty_area_dim[0]
            pen_area_half_y = self.field.penalty_area_dim[1]

            add_marker('l_lupa', -pen_area_x,  pen_area_half_y, 0)  # L-junction: left upper penalty area
            add_marker('l_llpa', -pen_area_x, -pen_area_half_y, 0)  # L-junction: left lower penalty area
            add_marker('l_rupa',  pen_area_x,  pen_area_half_y, 0)  # L-junction: right upper penalty area
            add_marker('l_rlpa',  pen_area_x, -pen_area_half_y, 0)  # L-junction: right lower penalty area

            add_marker('t_lupa', -field_half_x,  pen_area_half_y, 0)  # T-junction: left upper penalty area
            add_marker('t_llpa', -field_half_x, -pen_area_half_y, 0)  # T-junction: left lower penalty area
            add_marker('t_rupa',  field_half_x,  pen_area_half_y, 0)  # T-junction: right upper penalty area
            add_marker('t_rlpa',  field_half_x, -pen_area_half_y, 0)  # T-junction: right lower penalty area
        # fmt: on

        return world_spec

    def _request_participation(self, agent: PAgent, model_spec: Any) -> AgentID | None:
        # check player number
        if agent.get_player_no() > self.rules.max_player_no:
            return None

        # update known team names
        self.game_state.update_team_names(agent.get_team_name())

        # fetch team side for agent
        team_id = self.game_state.get_team_side(agent.get_team_name()).value

        if not TeamSide.is_valid(team_id):
            return None

        # check if a player with the same player number of the agent is already present in the game
        if agent.get_player_no() in self._team_players[team_id]:
            return None

        # check if the new player would exceed the maximum number of allowed players per team
        if len(self._team_players[team_id]) >= self.rules.max_team_size:
            return None

        agent_id = AgentID(team_id, agent.get_player_no())

        # append new player to team dict
        self._team_players[team_id][agent.get_player_no()] = SoccerPlayer(agent_id, model_spec)

        # set team color and spawn position
        root_body = model_spec.body('torso')
        root_body.first_geom().rgba = [0, 0, 1, 1] if team_id == TeamSide.LEFT.value else [1, 0, 0, 1]

        x_sign = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        root_body.pos[0] = x_sign * (2 * agent_id.player_no + 1)
        root_body.pos[1] = (self.field.field_dim[1] / 2) + self.field.field_border
        root_body.quat[0:4] = quat_from_axis_angle((0, 0, 1), -pi / 2)

        logger.debug('Spawn Team #%d Player #%02d @ (%.3f %.3f)', agent_id.team_id, agent_id.player_no, root_body.pos[0], root_body.pos[1])

        return agent_id

    def _handle_withdrawal(self, agent_id: AgentID) -> None:
        if TeamSide.is_valid(agent_id.team_id):
            with contextlib.suppress(KeyError):
                del self._team_players[agent_id.team_id][agent_id.player_no]

    def _post_step(self, monitor_commands: Sequence[MonitorCommand]) -> None:
        # update game objects
        self.ball.update(self._mj_model, self._mj_data)

        for p in self._team_players[TeamSide.LEFT.value].values():
            p.update(self._mj_model, self._mj_data)

        for p in self._team_players[TeamSide.RIGHT.value].values():
            p.update(self._mj_model, self._mj_data)

        # forward to parent
        super()._post_step(monitor_commands)

        # referee game
        self.referee.referee()

        # relocate objects as requested by referee
        self._relocate_objects()

    def _relocate_objects(self) -> None:
        """Relocate objects as requested by referee."""

        # relocate ball
        if self.ball.place_pos is not None:
            pos = (self.ball.place_pos[0], self.ball.place_pos[1], 0.12)
            rot = (1, 0, 0, 0)

            place_robot_3d('ball-', self._mj_model, self._mj_data, pos, rot)
            self.ball.last_position = pos
            self.ball.position = pos
            self.ball.place_pos = None

        # relocate players
        for players in self._team_players.values():
            for player in players.values():
                if player.place_pos is not None:
                    pos = (player.place_pos[0], player.place_pos[1], player.place_pos[2])
                    quat = (1.0, 0.0, 0.0, 0.0) if player.place_quat is None else player.place_quat

                    place_robot_3d(player.agent_id.prefix, self._mj_model, self._mj_data, pos, quat)
                    player.position = pos
                    player.place_pos = None
                    player.place_quat = None

    def _generate_game_state_perception(self) -> Perception:
        return GameStatePerception(
            play_time=self.game_state.get_play_time(),
            play_mode=self.game_state.get_play_mode().value,
            team_left=self.game_state.get_team_name(TeamSide.LEFT),
            team_right=self.game_state.get_team_name(TeamSide.RIGHT),
            score_left=self.game_state.get_team_score(TeamSide.LEFT),
            score_right=self.game_state.get_team_score(TeamSide.RIGHT),
        )

    def _generate_state_information(self) -> list[SimStateInformation]:
        state_info = super()._generate_state_information()

        state_info.insert(
            0,
            SoccerGameInformation(
                left_team=self.game_state.get_team_name(TeamSide.LEFT) or '<LEFT>',
                right_team=self.game_state.get_team_name(TeamSide.RIGHT) or '<RIGHT>',
                left_score=self.game_state.get_team_score(TeamSide.LEFT),
                right_score=self.game_state.get_team_score(TeamSide.RIGHT),
                play_time=self.game_state.get_play_time(),
                play_mode=self.game_state.get_play_mode().value,
            ),
        )

        if self._frame_id == 0:
            state_info.insert(0, SoccerEnvironmentInformation(self.field, self.rules))

        return state_info

    def beam_agent(self, actuator_name: str, beam_pose: tuple[float, float, float]) -> None:
        """Perform a beam action for the agent posing the given effector.

        Parameter
        ---------
        actuator_name: str
            The name of the beam actuator.

        beam_pose: tuple[float, float, float]
            The desired 2D beam pose (x, y, theta).
            Theta is given in radians.
        """

        # check if agents are allowed to beam
        if not self.referee.is_beaming_allowed():
            return

        # fetch agent id
        agent_id = AgentID.from_prefixed_name(actuator_name)
        if agent_id is None:
            return

        if not TeamSide.is_valid(agent_id.team_id):
            msg = 'Invalid team!'
            raise ValueError(msg)

        x_factor = -1 if agent_id.team_id == TeamSide.LEFT.value else 1
        theta_shift = 0 if agent_id.team_id == TeamSide.LEFT.value else pi
        pose = (abs(beam_pose[0]) * x_factor, beam_pose[1], beam_pose[2] + theta_shift)

        logger.debug('Beam Team #%d Player #%02d to (%.3f, %.3f, %.3f)', agent_id.team_id, agent_id.player_no, pose[0], pose[1], degrees(pose[2]))

        pos = (pose[0], pose[1], 0.6745)
        mujoco_quat = quat_from_axis_angle((0, 0, 1), pose[2])

        place_robot_3d(agent_id.prefix, self._mj_model, self._mj_data, pos, mujoco_quat)

    def request_kick_off(self, team_side: TeamSide | int) -> None:
        """Instruct kickoff for the given team.

        Parameter
        ---------
        team_side: TeamSide
            The team side for which to give the kick off.
        """

        self.referee.kick_off(team_side)

    def request_drop_ball(self, pos: tuple[float, float] | None = None) -> None:
        """Drop the ball at the specified position and instruct the normal progressing of the game.

        Parameter
        ---------
        pos: tuple[float, float] | None, default=None
            The position at which to drop the ball or none, to drop it at its current location.
        """

        self.referee.drop_ball(pos)

    def request_move_player(
        self,
        player_id: int,
        team_name: str,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Move the specified player to the specified position.

        Parameter
        ---------
        player_id: int
            The unique id of the player in its team.

        team_name: str
            The name of the team the player plays in or "Left" or "Right" for the left or the right team.

        pos: tuple[float, float, float]
            The position to which to move the player.

        quat: tuple[float, float, float, float] | None, default=None
            The 3D rotation quaternion of the torso.
        """

        self.referee.move_player(player_id, team_name, pos, quat)
