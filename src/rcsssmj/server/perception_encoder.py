from abc import ABC, abstractmethod
from collections.abc import Sequence

from rcsssmj.sim.perceptions import Perception


class PerceptionEncoder(ABC):
    """Base class for perception message encoders."""

    @abstractmethod
    def encode(self, perceptions: Sequence[Perception]) -> bytes | bytearray | None:
        """Encode the given sequence of perceptions into a message."""


class DefaultPerceptionEncoder(PerceptionEncoder):
    """Default perception message encoder implementation based on symbolic expressions."""

    def encode(self, perceptions: Sequence[Perception]) -> bytes | bytearray | None:
        """Encode the given sequence of perceptions into a symbolic expression message."""

        msg = ''.join([p.to_sexp() for p in perceptions])

        return msg.encode('utf-8')
