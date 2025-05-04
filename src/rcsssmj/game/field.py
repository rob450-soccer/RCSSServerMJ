from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum

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

    # cases: FIFA and UNKNOWN
    return FIFASoccerField()


@dataclass
class SoccerField:
    """Representation of a soccer field.

    All dimensions in this description are related to a coordinate system, in which positive x points towards the right goal, positive z towards the sky and positive y according to the right hand rule:

    Note: Use default values from official FIFA rule book if there exists an equivalent rule. In all other cases try to choose sensible values which work in conjunction with the official FIFA rule book.
    """

    field_dim: tuple[float, float, float] = (105, 68, 40)  # officially between 100 x 64 and 110 x 75 meters (105 x 68 is recommended) - z height is arbitrary
    """The soccer field dimensions, where positive x points towards the right goal, positive z towards the sky and positive y according to the right hand rule."""

    line_width: float = 0.1  # officially max width of 12cm, but at least the goal line has to match the diameter of the goal posts
    """The width of the field lines."""

    field_border: float = 3
    """The size of the border around the field."""

    goal_dim: tuple[float, float, float] = (1.6, 7.32, 2.44)  # could not find official specification of depth (x dimensions)
    """The inner soccer goal dimensions without posts and crossbar."""

    goal_post_radius: float = 0.05  # officially max diameter of 12cm, but has to be the same as the goal line width
    """The radius of the soccer goal posts and crossbar (according to official FIFA rules, the diameter of the goal posts have to match the width of the goal line)."""

    penalty_area_dim: tuple[float, float] | None = (5.5, 7.32 + 2 * 5.5)  # official size
    """The dimensions of the penalty area (if existing)."""

    goalie_area_dim: tuple[float, float] = (16.5, 7.32 + 2 * 16.5)  # official size
    """The dimensions of the goalie area (if existing)."""

    corner_area_radius: float = 1  # official radius
    """The radius of the corner area."""

    penalty_spot_distance: float = 11  # well known official penalty spot
    """The distance of the penalty spot from the goal line."""

    center_circle_radius: float = 9.15  # official radius
    """The radius of the center circle."""


@dataclass
class FIFASoccerField(SoccerField):
    """Official FIFA soccer field specification."""


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
            penalty_area_dim=None,  # none specified
            goalie_area_dim=(1, 2.6 + 2 * 1.2),
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
            penalty_area_dim=None,  # none specified
            goalie_area_dim=(1, 2.6 + 2 * 1.2),
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
            penalty_area_dim=(3, 2.6 + 2 * 1.7),
            goalie_area_dim=(1, 2.6 + 2 * 0.7),
            corner_area_radius=0,  # none specified
            penalty_spot_distance=2.1,
            center_circle_radius=1.5,
        )
