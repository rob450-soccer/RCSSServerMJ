import logging
from typing import TYPE_CHECKING, Final

from websockets import ConnectionClosed
from websockets.sync.connection import Connection

if TYPE_CHECKING:
    import socket

logger = logging.getLogger(__name__)


class WSConnection:
    """Websocket based connection for receiving and sending messages."""

    def __init__(
        self,
        conn: Connection,
    ) -> None:
        """Construct a new websocket connection.

        Parameter
        ---------
        conn: Connection
            The websocket connection.
        """

        self.conn: Final[Connection] = conn
        """The websocket connection."""

        self.addr: Final[socket.AddressInfo] = conn.remote_address
        """The websocket address information."""

        self._active: bool = True
        """Flag if the websocket is active or not."""

    def is_active(self) -> bool:
        """Check if the connection is still active."""

        return self._active

    def send_message(self, msg: bytes | bytearray) -> None:
        """Send the given message."""

        if self._active:
            self.conn.send(msg)

    def receive_message(self) -> bytes | bytearray:
        """Receive the next message from the websocket."""

        if not self._active:
            raise ConnectionResetError

        try:
            return self.conn.recv(None, False)
        except ConnectionClosed as err:
            raise ConnectionResetError from err

    def shutdown(self) -> None:
        """Shutdown / close this connection."""

        self._active = False

        try:
            self.conn.close()
        except Exception:  # noqa: BLE001
            logger.debug('ERROR shutting down websocket connection!', exc_info=True)

    def close(self) -> None:
        """Close this connection."""

        self._active = False

        try:
            self.conn.close()
        except Exception:  # noqa: BLE001
            logger.debug('ERROR closing websocket connection!', exc_info=True)

    def __str__(self) -> str:
        return f'WS{self.addr}'

    def __repr__(self) -> str:
        return f'WSConnection({self.addr}'
