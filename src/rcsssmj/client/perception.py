from abc import ABC, abstractmethod
from typing import Final


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
