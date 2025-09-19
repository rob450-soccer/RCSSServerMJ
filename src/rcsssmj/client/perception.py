from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Final, Protocol


class Perception(ABC):
    """Base class for perceptions."""

    def __init__(self, name: str) -> None:
        """Construct a new perception.

        Parameter
        ---------
        name: str
            The name of the sensor that generated the perception.
        """

        self.name: Final[str] = name
        """The sensor / perceptor name."""

    @abstractmethod
    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception."""


class TimePerception(Perception):
    """Time perception."""

    def __init__(self, name: str, time: float) -> None:
        """Construct a new time perception.

        Parameter
        ---------
        name: str
            The name of the sensor that generated the perception.

        time: float
            The perceived time.
        """

        super().__init__(name)

        self.time: Final[float] = time
        """The perceived time."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (time (<name> <time>))
        """

        return f'(time ({self.name} {self.time}))'


class PositionPerception(Perception):
    """Position perception."""

    def __init__(self, name: str, x: float, y: float, z: float) -> None:
        """Construct a new position perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        x: float
            The x-coordinate of the perceived position.

        y: float
            The y-coordinate of the perceived position.

        z: float
            The z-coordinate of the perceived position.
        """

        super().__init__(name)

        self.x: Final[float] = x
        """The x-coordinate of the position."""

        self.y: Final[float] = y
        """The y-coordinate of the position."""

        self.z: Final[float] = z
        """The z-coordinate of the position."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (pos (n <name>) (p <x> <y> <z>))
        """

        return f'(pos (n {self.name}) (p {self.x} {self.y} {self.z}))'


class OrientationPerception(Perception):
    """Orientation perception."""

    def __init__(self, name: str, qw: float, qx: float, qy: float, qz: float) -> None:
        """Construct a new orientation perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        qw: float
            The w-coordinate of the orientation quaternion.

        qx: float
            The x-coordinate of the orientation quaternion.

        qy: float
            The y-coordinate of the orientation quaternion.

        qz: float
            The z-coordinate of the orientation quaternion.
        """

        super().__init__(name)

        self.qw: Final[float] = qw
        """The w-coordinate of the orientation quaternion."""

        self.qx: Final[float] = qx
        """The x-coordinate of the orientation quaternion."""

        self.qy: Final[float] = qy
        """The y-coordinate of the orientation quaternion."""

        self.qz: Final[float] = qz
        """The z-coordinate of the orientation quaternion."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (quat (n <name>) (q <qw> <qx> <qy> <qz>))
        """

        return f'(quat (n {self.name}) (q {self.qw} {self.qx} {self.qy} {self.qz}))'


class JointStatePerception(Perception):
    """Sequence of joint perceptions."""

    def __init__(self, joint_names: Sequence[str], axs: Sequence[float], vxs: Sequence[float]) -> None:
        """Construct a new joint state perception.

        Parameter
        ---------
        joint_names: Sequence[str]
            The list of joint names.

        axs: Sequence[float]
            The list of joint angles.

        vxs: Sequence[float]
            The list of joint velocities.
        """

        super().__init__('JS')

        self.joint_names: Sequence[str] = joint_names
        """The list of joint names."""

        self.joint_axs: Sequence[float] = axs
        """THe list of joint axis positions."""

        self.joint_vxs: Sequence[float] = vxs
        """The list of joint velocities."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (HJ (n <name>) (ax <ax>), (vx <vx>))*
        """

        return ''.join(f'(HJ (n {name})(ax {ax})(vx {vx}))' for name, ax, vx in zip(self.joint_names, self.joint_axs, self.joint_vxs, strict=False))

    def to_sexp2(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (JS (names <name1> <name2> ...) (axs <ax1> <ax2> ...) (vxs <vx1> <vx2> ...))
        """

        names = ' '.join(self.joint_names)
        axs = ' '.join(str(ax) for ax in self.joint_axs)
        vxs = ' '.join(str(vx) for vx in self.joint_vxs)

        return f'({self.name} (names {names}) (axs {axs}) (vxs {vxs}))'


class GyroPerception(Perception):
    """Gyroscope perception."""

    def __init__(self, name: str, rx: float, ry: float, rz: float) -> None:
        """Construct a new gyroscope perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        rx: float
            The rotational velocity around the x axis.

        ry: float
            The rotational velocity around the y axis.

        rz: float
            The rotational velocity around the z axis.
        """

        super().__init__(name)

        self.rx: Final[float] = rx
        """The rotational velocity around the x-axis."""

        self.ry: Final[float] = ry
        """The rotational velocity around the y-axis."""

        self.rz: Final[float] = rz
        """The rotational velocity around the z-axis."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (GYR (n <name>) (rt <x> <y> <z>))
        """

        return f'(GYR (n {self.name}) (rt {self.rx} {self.ry} {self.rz}))'


class AccelerometerPerception(Perception):
    """Accelerometer perception."""

    def __init__(self, name: str, ax: float, ay: float, az: float) -> None:
        """Construct a new gyroscope perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        ax: float
            The acceleration along the x-axis.

        ay: float
            The acceleration along the y-axis.

        az: float
            The acceleration along the z-axis.
        """

        super().__init__(name)

        self.ax: Final[float] = ax
        """The linear acceleration along the x-axis."""

        self.ay: Final[float] = ay
        """The linear acceleration along the y-axis."""

        self.az: Final[float] = az
        """The linear acceleration along the z-axis."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (ACC (n <name>) (a <x> <y> <z>))
        """

        return f'(ACC (n {self.name}) (a {self.ax} {self.ay} {self.az}))'


class TouchPerception(Perception):
    """Touch perception."""

    def __init__(self, name: str, active: int) -> None:
        """Construct a new gyroscope perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        active: int
            Flag if the touch perceptor has active contact.
        """

        super().__init__(name)

        self.active: Final[int] = active
        """Flag if bumper is active (!= 0) or inactive (== 0)"""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (TCH n <name> val <active>)
        """

        return f'(TCH n {self.name} val {self.active})'


class PObjectDetection(Protocol):
    """Base protocol for all object detections of a vision sensor-pipeline."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this detection."""


class ObjectDetection:
    """Simple class for holding an object detection of a vision sensor-pipeline.

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
            The inclination / elevation (vertical) angle.

        azimuth : float
            The azimuth (horizontal) angle.
        """

        self.name: Final[str] = name
        """The name of the object / reference frame."""

        self.azimuth: Final[float] = azimuth
        """The azimuth (horizontal) angle to the detected point."""

        self.inclination: Final[float] = inclination
        """The elevation (vertical) angle to the detected point."""

        self.distance: Final[float] = distance
        """The distance to the detected point (in meter)."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (<name> (pol <distance> <azimuth> <inclination>))
        """

        return f'({self.name} (pol {self.distance} {self.azimuth} {self.inclination}))'


class AgentDetection:
    """Simple class for holding an object detection of a vision sensor-pipeline.

    An agent detection consists of the team name and player number of the agent together with a list of object detections for individual body parts / visual markers of the agent.
    """

    def __init__(self, name: str, team_name: str, player_no: int, body_detections: Sequence[ObjectDetection]) -> None:
        """Construct a new object detection.

        Parameter
        ---------
        name: str
            The object / detection class.

        team_name: str
            The name of the team the detected agent belongs to.

        player_no: int
            The player number of the detected agent.

        body_detections: Sequence[ObjectDetection]
            The list of individual agent body object detections.
        """

        self.name: Final[str] = name
        """The agent Identifier 'P'."""

        self.team_name: Final[str] = team_name
        """The name of the team the player belongs to."""

        self.player_no: Final[int] = player_no
        """The player number."""

        self.body_detections: Final[Sequence[ObjectDetection]] = body_detections
        """The list of individual body part detections."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (<name> (team <team-name>) (id <player-no>) [(<marker> (pol <distance> <h-angle> <v-angle>))])
        """

        return '(' + self.name + ' (team ' + self.team_name + ')(id ' + str(self.player_no) + ')' + ''.join(detection.to_sexp() for detection in self.body_detections) + ')'


class VisionPerception(Perception):
    """Vision sensor-pipeline perception."""

    def __init__(self, name: str, objects: Sequence[PObjectDetection]) -> None:
        """Construct a new vision sensor-pipeline perception.

        Parameter
        ---------
        name: str
            The sensor / perceptor name.

        objects: Sequence[PObjectDetection]
            The list of object detections.
        """

        super().__init__(name)

        self.obj_detections: Final[Sequence[PObjectDetection]] = objects
        """The list of detected objects."""

    def to_sexp(self) -> str:
        """Return an symbolic expression representing this perception.

        Expression format: (See ...)
        """

        detections = [d.to_sexp() for d in self.obj_detections]

        return '(' + self.name + ' ' + ''.join(detections) + ')'
