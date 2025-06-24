from typing import Protocol, runtime_checkable


@runtime_checkable
class PGame(Protocol):
    """Protocol for a game."""

    @property
    def sim_time(self) -> float:
        """The current simulation time."""
