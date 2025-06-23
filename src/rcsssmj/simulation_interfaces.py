from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class PSimActionInterface(Protocol):
    """Protocol for a soccer action interface."""

    @property
    def mj_model(self) -> Any:
        """The current mujoco simulation model."""

    @property
    def mj_data(self) -> Any:
        """The current mujoco simulation data array."""


@runtime_checkable
class PSimCommandInterface(Protocol):
    """Protocol for the soccer command interface."""

    def kill_sim(self) -> None:
        """Kill the simulation (server)."""
