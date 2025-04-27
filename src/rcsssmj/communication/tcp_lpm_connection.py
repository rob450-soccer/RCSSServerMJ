import socket
from typing import Final


class TCPLPMConnection:
    """
    TCP/IP based connection for receiving and sending (typically 4-byte) length prefixed messages.
    """

    def __init__(
        self,
        sock: socket.socket,
        addr: socket.AddressInfo,
        length_prefix_size: int = 4,
    ) -> None:
        """
        Construct a new TCP/IP LPM connection.
        """

        self.sock: Final[socket.socket] = sock
        self.addr: Final[socket.AddressInfo] = addr

        self._active: bool = True

        self.length_prefix_size: Final[int] = length_prefix_size
        self._rcv_buffer_size = 1024
        self._rcv_buffer = bytearray(self._rcv_buffer_size)

        # set TCP_NODELAY option to send messages immediately (without buffering)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def is_active(self) -> bool:
        """
        Check if the connection is still active.
        """

        return self._active

    def send_message(self, msg: bytes | bytearray) -> None:
        """
        Send the given message.
        """

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
        """
        Shutdown / close this connection.
        """

        self._active = False

        # close and clear socket on shutdown
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except:
            print('ERROR shutting down socket!')

    def close(self) -> None:
        """
        Close this connection.
        """

        self._active = False

        # close and clear socket
        self.sock.close()
