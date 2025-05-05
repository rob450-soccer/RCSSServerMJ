from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Final, Protocol


class Perception(ABC):
    """
    Base class for perceptions.
    """

    def __init__(self, name: str) -> None:
        """
        Construct a new perception.
        """

        self.name: Final[str] = name

    @abstractmethod
    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.
        """


class TimePerception(Perception):
    """
    A time perception.
    """

    def __init__(self, name: str, time: float) -> None:
        """
        Construct a new time perception.
        """

        super().__init__(name)

        self.time: Final[float] = time

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (time (<name> <time>))
        """

        return f'(time ({self.name} {self.time}))'


class PositionPerception(Perception):
    """
    A position perception.
    """

    def __init__(self, name: str, x: float, y: float, z: float) -> None:
        """
        Construct a new position perception.
        """

        super().__init__(name)

        self.x: Final[float] = x
        self.y: Final[float] = y
        self.z: Final[float] = z

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (pos (name <name>) (p <x> <y> <z>))
        """

        return f'(pos (name {self.name}) (p {self.x} {self.y} {self.z}))'


class OrientationPerception(Perception):
    """
    An orientation perception.
    """

    def __init__(self, name: str, qw: float, qx: float, qy: float, qz: float) -> None:
        """
        Construct a new orientation perception.
        """

        super().__init__(name)

        self.qw: Final[float] = qw
        self.qx: Final[float] = qx
        self.qy: Final[float] = qy
        self.qz: Final[float] = qz

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (quat (name <name>) (q <qw> <qx> <qy> <qz>))
        """

        return f'(quat (name {self.name}) (q {self.qw} {self.qx} {self.qy} {self.qz}))'


class JointPerception(Perception):
    """
    A joint perception.
    """

    def __init__(self, name: str, ax: float) -> None:
        """
        Construct a new joint perception.
        """

        super().__init__(name)

        self.ax: Final[float] = ax

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (HJ (name <name>) (ax <ax>))
        """

        return f'(HJ (name {self.name}) (ax {self.ax}))'


class GyroPerception(Perception):
    """
    A gyroscope perception.
    """

    def __init__(self, name: str, rx: float, ry: float, rz: float) -> None:
        """
        Construct a new gyroscope perception.
        """

        super().__init__(name)

        self.rx: Final[float] = rx
        self.ry: Final[float] = ry
        self.rz: Final[float] = rz

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (GYR (name <name>) (rt <rx> <ry> <rz>))
        """

        return f'(GYR (name {self.name}) (rt {self.rx} {self.ry} {self.rz}))'


class AccelerometerPerception(Perception):
    """
    A accelerometer perception.
    """

    def __init__(self, name: str, ax: float, ay: float, az: float) -> None:
        """
        Construct a new gyroscope perception.
        """

        super().__init__(name)

        self.ax: Final[float] = ax
        self.ay: Final[float] = ay
        self.az: Final[float] = az

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.
        """

        return f'(ACC (name {self.name}) (a {self.ax} {self.ay} {self.az}))'


class TouchPerception(Perception):
    """
    A touch perception.
    """

    def __init__(self, name: str, active: int) -> None:
        """
        Construct a new gyroscope perception.
        """

        super().__init__(name)

        self.active: Final[int] = active

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (TCH <name> val <active>)
        """

        return f'(TCH name {self.name} val {self.active})'


class GameStatePerception(Perception):
    """
    A game state perception.
    """

    def __init__(
        self,
        score_left: int,
        score_right: int,
        play_time: float,
        play_mode: str,
    ) -> None:
        """
        Construct a new game state perception.
        """

        super().__init__('GS')

        self.score_left: Final[int] = score_left
        self.score_right: Final[int] = score_right
        self.play_time: Final[float] = play_time
        self.play_mode: Final[str] = play_mode

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (GS (sl <sl>) (sr <sr>) (t <play_time>) (pm <play_mode>))
        """

        return f'(GS (sl {self.score_left}) (sr {self.score_right}) (t {self.play_time}) (pm {self.play_mode}))'


class PObjectDetection(Protocol):
    """
    Base protocol for all object detections of a vision sensor-pipeline.
    """

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this detection.
        """


class ObjectDetection:
    """
    Simple class for holding an object detection of a vision sensor-pipeline.

    An object detection consists of a single point relating to the center of the object.
    """

    def __init__(self, name: str, azimuth: float, inclination: float, distance: float) -> None:
        """Construct a new object detection.

        Parameter
        ---------
        name : str
            The name of the object / detection.

        azimuth : float
            The azimuth (horizontal) angle.

        inclination : float
            The inclination / elevation (vertival) angle.

        azimuth : float
            The azimuth (horizontal) angle.
        """

        self.name: Final[str] = name
        self.azimuth: Final[float] = azimuth
        self.inclination: Final[float] = inclination
        self.distance: Final[float] = distance

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (<name> (pol <azimuth> <inclination> <distance>))
        """

        return f'({self.name} (pol {self.azimuth} {self.inclination} {self.distance}))'


class AgentDetection:
    """
    Simple class for holding an object detection of a vision sensor-pipeline.

    An agent detection consists of the team name and player number of the agent together with a list of object detections for individual body parts / visual markers of the agent.
    """

    def __init__(self, name: str, team_name: str, player_no: int, body_detections: Sequence[ObjectDetection]) -> None:
        """Construct a new object detection."""

        self.name: Final[str] = name
        self.team_name: Final[str] = team_name
        self.player_no: Final[int] = player_no
        self.body_detections: Final[Sequence[ObjectDetection]] = body_detections

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (<name> (team <team-name>) (id <player-no>) [(<marker> (pol <h-angle> <v-angle> <distance>))])
        """

        return '(' + self.name + ' (team ' + self.team_name + ')(id ' + str(self.player_no) + ')' + ''.join(detection.to_sexp() for detection in self.body_detections) + ')'


class VisionPerception(Perception):
    """
    An vision sensor-pipeline perception.
    """

    def __init__(self, name: str, objects: Sequence[PObjectDetection]) -> None:
        """
        Construct a new vision sensor-pipeline perception.
        """

        super().__init__(name)

        self.obj_detections: Final[Sequence[PObjectDetection]] = objects

    def to_sexp(self) -> str:
        """
        Return an symbolic expression representing this perception.

        Expression format: (See ...)
        """

        detections = [d.to_sexp() for d in self.obj_detections]

        return '(' + self.name + ' ' + ''.join(detections) + ')'
