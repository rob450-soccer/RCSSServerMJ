from typing import Protocol


class PConnection(Protocol):
    """Base protocol for message based connections."""

    def is_active(self) -> bool:
        """Check if the connection is still active."""

    def send_message(self, msg: bytes | bytearray) -> None:
        """Send the given message.

        Parameter
        ---------
        msg: bytes | bytearray
            The message to send.
        """

    def receive_message(self) -> bytes | bytearray:
        """Receive the next message."""

    def shutdown(self) -> None:
        """Request connection shutdown."""

    def close(self) -> None:
        """Close this connection."""
