from abc import ABC, abstractmethod
from collections.abc import Sequence

from rcsssmj.client.perception import Perception


class PerceptionEncoder(ABC):
    """
    Base class for perception message encoders.
    """

    @abstractmethod
    def encode(self, perceptions: Sequence[Perception]) -> bytes | bytearray | None:
        """
        Encode the given sequence of perceptions into a message.
        """


class SExprPerceptionEncoder(PerceptionEncoder):
    """
    Default symbolic expression perception message encoder implementation.
    """

    def encode(self, perceptions: Sequence[Perception]) -> bytes | bytearray | None:
        """
        Encode the given sequence of perceptions into a symbolic expression message.
        """

        msg = ''.join([p.to_sexp() for p in perceptions])

        return msg.encode('utf-8')
