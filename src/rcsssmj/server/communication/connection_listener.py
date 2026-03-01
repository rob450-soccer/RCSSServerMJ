from typing import Protocol


class PConnectionListener(Protocol):
    """Protocol for connection listeners."""

    def bind(self) -> None:
        """Setup connection interface."""

    def listen_for_connections(self) -> None:
        """Start listening for incoming connections."""

    def is_alive(self) -> bool:
        """Return whether the connection listener is alive."""

    def join(self) -> None:
        """Wait until the listener thread terminates (if existing)."""

    def shutdown(self) -> None:
        """Shutdown connection listener."""
