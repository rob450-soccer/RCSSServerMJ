from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum

from rcsssmj.utils.geometry import AABB2D, AABB3D

logger = logging.getLogger(__name__)


class SoccerFieldVersions(Enum):
    """Enum specifying available soccer field versions."""

    UNKNOWN = 'unknown'
    """Unknown field version (only used to indicate an error)."""

    FIFA = 'fifa'
    """Official FIFA soccer field."""

    HL_ADULT = 'hl_adult'
    """Latest version of the RoboCup Humanoid Adult Size League soccer field."""

    HL_ADULT_2014 = 'hl_adult_2014'
    """2014 version of the RoboCup Humanoid Adult Size League soccer field."""

    HL_ADULT_2019 = 'hl_adult_2019'
    """2019 version of the RoboCup Humanoid Adult Size League soccer field."""

    HL_ADULT_2020 = 'hl_adult_2020'
    """2020 version of the RoboCup Humanoid Adult Size League soccer field."""

    SIM3D_7VS7 = 'sim3d_7vs7'
    """3D simulation version of the 7 vs 7 soccer field."""

    @staticmethod
    def from_value(version: str) -> SoccerFieldVersions:
        """Fetch the enum entry corresponding to the given version value."""

        for v in SoccerFieldVersions:
            if v.value == version:
                return v

        logger.warning('Unknown field version: %s!', version)

        return SoccerFieldVersions.UNKNOWN


def create_soccer_field(version: str) -> SoccerField:
    """Create a soccer field representation for the given field version."""

    version_id = SoccerFieldVersions.from_value(version)

    if version_id == SoccerFieldVersions.HL_ADULT_2014:
        return HLAdult2014SoccerField()
    if version_id == SoccerFieldVersions.HL_ADULT_2019:
        return HLAdult2019SoccerField()
    if version_id in (SoccerFieldVersions.HL_ADULT_2020, SoccerFieldVersions.HL_ADULT):
        return HLAdult2020SoccerField()
    if version_id == SoccerFieldVersions.SIM3D_7VS7:
        return Sim3D7vs7SoccerField()

    # cases: FIFA and UNKNOWN
    return FIFASoccerField()


@dataclass
class SoccerField:
    """Representation of a soccer field.

    All dimensions in this description are related to a coordinate system, in which positive x points towards the right goal, positive z towards the sky and positive y according to the right hand rule:

    Note: Use default values from official FIFA rule book if there exists an equivalent rule. In all other cases try to choose sensible values which work in conjunction with the official FIFA rule book.
    """

    field_dim: tuple[float, float, float]
    """The soccer field dimensions, where positive x points towards the right goal, positive z towards the sky and positive y according to the right hand rule."""

    line_width: float
    """The width of the field lines."""

    field_border: float
    """The size of the border around the field."""

    goal_dim: tuple[float, float, float]
    """The inner soccer goal dimensions without posts and crossbar."""

    goal_post_radius: float
    """The radius of the soccer goal posts and crossbar (according to official FIFA rules, the diameter of the goal posts have to match the width of the goal line)."""

    goalie_area_dim: tuple[float, float]
    """The dimensions of the goalie area."""

    penalty_area_dim: tuple[float, float] | None
    """The dimensions of the penalty area (if existing)."""

    corner_area_radius: float
    """The radius of the corner area."""

    penalty_spot_distance: float
    """The distance of the penalty spot from the goal line."""

    center_circle_radius: float
    """The radius of the center circle."""

    field_area: AABB2D
    """The soccer field area."""

    left_goal_box: AABB3D
    """The left goal volume."""

    right_goal_box: AABB3D
    """The left goal volume."""

    left_goalie_area: AABB2D
    """The left goalie area."""

    right_goalie_area: AABB2D
    """The right goalie area."""

    left_penalty_area: AABB2D | None
    """The left penalty area."""

    right_penalty_area: AABB2D | None
    """The right penalty area."""

    def __init__(
        self,
        field_dim: tuple[float, float, float],
        line_width: float,
        field_border: float,
        goal_dim: tuple[float, float, float],
        goal_post_radius: float,
        goalie_area_dim: tuple[float, float],
        penalty_area_dim: tuple[float, float] | None,
        corner_area_radius: float,
        penalty_spot_distance: float,
        center_circle_radius: float,
    ) -> None:
        """Construct a new soccer field."""

        self.field_dim = field_dim
        self.line_width = line_width
        self.field_border = field_border
        self.goal_dim = goal_dim
        self.goal_post_radius = goal_post_radius
        self.goalie_area_dim = goalie_area_dim
        self.penalty_area_dim = penalty_area_dim
        self.corner_area_radius = corner_area_radius
        self.penalty_spot_distance = penalty_spot_distance
        self.center_circle_radius = center_circle_radius

        field_half_x = field_dim[0] / 2
        field_half_y = field_dim[1] / 2
        self.field_area = AABB2D(-field_half_x, field_half_x, -field_half_y, field_half_y)

        goal_half_y = goal_dim[1] / 2
        self.left_goal_box = AABB3D(-field_half_x - goal_dim[0], -field_half_x, -goal_half_y, goal_half_y, 0, goal_dim[2])
        self.right_goal_box = AABB3D(field_half_x, field_half_x + goal_dim[0], -goal_half_y, goal_half_y, 0, goal_dim[2])

        ga_half_y = goalie_area_dim[1] / 2
        self.left_goalie_area = AABB2D(-field_half_x, -field_half_x + goalie_area_dim[0], -ga_half_y, ga_half_y)
        self.right_goalie_area = AABB2D(field_half_x - goalie_area_dim[0], field_half_x, -ga_half_y, ga_half_y)

        if penalty_area_dim is not None:
            pa_half_y = penalty_area_dim[1] / 2
            self.left_penalty_area = AABB2D(-field_half_x, -field_half_x + penalty_area_dim[0], -pa_half_y, pa_half_y)
            self.right_penalty_area = AABB2D(field_half_x - penalty_area_dim[0], field_half_x, -pa_half_y, pa_half_y)
        else:
            self.left_penalty_area = None
            self.right_penalty_area = None


@dataclass
class FIFASoccerField(SoccerField):
    """Official FIFA soccer field specification."""

    def __init__(self) -> None:
        """Construct a new FIFA soccer field."""

        super().__init__(
            field_dim=(105, 68, 40),  # officially between 100 x 64 and 110 x 75 meters (105 x 68 is recommended) - z height is arbitrary
            line_width=0.1,  # officially max width of 12cm, but at least the goal line has to match the diameter of the goal posts
            field_border=3.0,
            goal_dim=(1.6, 7.32, 2.44),  # could not find official specification of depth (x dimensions)
            goal_post_radius=0.05,  # officially max diameter of 12cm, but has to be the same as the goal line width
            goalie_area_dim=(5.5, 7.32 + 2 * 5.5),  # official size
            penalty_area_dim=(16.5, 7.32 + 2 * 16.5),  # official size
            corner_area_radius=1,  # official radius
            penalty_spot_distance=11,  # well known official penalty spot
            center_circle_radius=9.15,  # official radius
        )


@dataclass
class HLAdult2014SoccerField(SoccerField):
    """Official RoboCup Humanoid Adult Size League soccer field specification used from 2014 until 2018."""

    def __init__(self) -> None:
        """Construct a new RCHL-Adult 2014 soccer field description."""

        super().__init__(
            field_dim=(9, 6, 40),
            line_width=0.05,
            field_border=2,  # min 0.7m
            goal_dim=(0.6, 2.6, 1.8),
            goal_post_radius=0.05,  # new in 2017: diameter <= 12cm
            goalie_area_dim=(1, 2.6 + 2 * 1.2),
            penalty_area_dim=None,  # none specified
            corner_area_radius=0,  # none specified
            penalty_spot_distance=2.1,
            center_circle_radius=0.75,
        )


@dataclass
class HLAdult2019SoccerField(SoccerField):
    """Official RoboCup Humanoid Adult Size League soccer field specification used in 2019."""

    def __init__(self) -> None:
        """Construct a new RCHL-Adult 2019 soccer field description."""

        super().__init__(
            field_dim=(14, 9, 40),
            line_width=0.05,
            field_border=2,  # min 1m
            goal_dim=(0.6, 2.6, 1.8),
            goal_post_radius=0.05,  # diameter <= 12cm
            goalie_area_dim=(1, 2.6 + 2 * 1.2),
            penalty_area_dim=None,  # none specified
            corner_area_radius=0,  # none specified
            penalty_spot_distance=2.1,
            center_circle_radius=1.5,
        )


@dataclass
class HLAdult2020SoccerField(SoccerField):
    """Official RoboCup Humanoid Adult Size League soccer field specification used since 2020 until now."""

    def __init__(self) -> None:
        """Construct a new RCHL-Adult 2020 soccer field description."""

        super().__init__(
            field_dim=(14, 9, 40),
            line_width=0.05,
            field_border=2,  # min 1m
            goal_dim=(0.6, 2.6, 1.8),
            goal_post_radius=0.05,  # 8cm <= diameter <= 12cm
            goalie_area_dim=(1, 2.6 + 2 * 0.7),
            penalty_area_dim=(3, 2.6 + 2 * 1.7),
            corner_area_radius=0,  # none specified
            penalty_spot_distance=2.1,
            center_circle_radius=1.5,
        )


@dataclass
class Sim3D7vs7SoccerField(SoccerField):
    """Official FIFA soccer field specification for 7 vs 7 games."""

    def __init__(self) -> None:
        """Construct a new FIFA soccer field."""

        super().__init__(
            field_dim=(55, 36, 40),  # official  7vs7 measures - z height is arbitrary
            line_width=0.1,  # officially max width of 12cm, but at least the goal line has to match the diameter of the goal posts
            field_border=3.0,
            goal_dim=(1, 3.66, 1.83),  # could not find official specification of depth (x dimensions)
            goal_post_radius=0.05,  # officially max diameter of 12cm, but has to be the same as the goal line width
            goalie_area_dim=(4, 7.3),  # adjusted from official penalty area size
            penalty_area_dim=(9, 16.5),  # official size
            corner_area_radius=1,  # not sure if used
            penalty_spot_distance=7.32,  # official penalty spot
            center_circle_radius=5.5,  # official radius
        )
