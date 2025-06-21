import logging
import socket
from typing import Final

logger = logging.getLogger(__name__)


class TCPLPMConnection:
    """TCP/IP based connection for receiving and sending (typically 4-byte) length prefixed messages."""

    def __init__(
        self,
        sock: socket.socket,
        addr: socket.AddressInfo,
        length_prefix_size: int = 4,
    ) -> None:
        """Construct a new TCP/IP LPM connection.

        Parameter
        ---------
        sock: socket.socket
            The TCP/IP socket.

        addr: socket.AddressInfo
            The socket address info.

        length_prefix_size: int = 4
            The size of the message length prefix.
        """

        self.sock: Final[socket.socket] = sock
        """The TCP/IP socket."""

        self.addr: Final[socket.AddressInfo] = addr
        """The socket address information."""

        self._active: bool = True
        """Flag if the socket is active or not."""

        self.length_prefix_size: Final[int] = length_prefix_size
        """The size (in number of bytes) of the message length prefix."""

        self._rcv_buffer_size = 1024
        """The current total size of the receive buffer."""

        self._rcv_buffer = bytearray(self._rcv_buffer_size)
        """Buffer for receiving messages."""

        # set TCP_NODELAY option to send messages immediately (without buffering)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def is_active(self) -> bool:
        """Check if the connection is still active."""

        return self._active

    def send_message(self, msg: bytes | bytearray) -> None:
        """Send the given message."""

        if self._active:
            self.sock.send((len(msg)).to_bytes(self.length_prefix_size, byteorder='big') + msg)

    def receive_message(self) -> bytes | bytearray:
        """
        Receive the next message from the TCP/IP socket.
        """

        if not self._active:
            raise ConnectionResetError

        # receive message length information
        if self.sock.recv_into(self._rcv_buffer, nbytes=self.length_prefix_size, flags=socket.MSG_WAITALL) != self.length_prefix_size:
            self._active = False
            raise ConnectionResetError

        msg_size = int.from_bytes(self._rcv_buffer[: self.length_prefix_size], byteorder='big', signed=False)

        # ensure receive buffer is large enough to hold the message
        if msg_size > self._rcv_buffer_size:
            self._rcv_buffer_size = msg_size
            self._rcv_buffer = bytearray(self._rcv_buffer_size)

        # receive message with the specified length
        if self.sock.recv_into(self._rcv_buffer, nbytes=msg_size, flags=socket.MSG_WAITALL) != msg_size:
            self._active = False
            raise ConnectionResetError

        return self._rcv_buffer[:msg_size]

    def shutdown(self) -> None:
        """Shutdown / close this connection."""

        self._active = False

        # close and clear socket on shutdown
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:  # noqa: BLE001
            logger.debug('ERROR shutting down socket!', exc_info=True)

    def close(self) -> None:
        """Close this connection."""

        self._active = False

        # close and clear socket
        self.sock.close()

    def __str__(self) -> str:
        return str(self.addr)
