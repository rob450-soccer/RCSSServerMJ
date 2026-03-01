import logging
import socket
from collections.abc import Callable
from threading import Thread
from typing import Final

from rcsssmj.server.communication.connection import PConnection
from rcsssmj.server.communication.tcp_lpm_connection import TCPLPMConnection

logger = logging.getLogger(__name__)


class TCPConnectionListener:
    """Helper class listening for incoming TCP/IP connections on a specified interface and port.

    Incoming connection requests are forwarded to a user defined handler function.
    """

    def __init__(
        self,
        handler: Callable[[PConnection], None],
        name: str,
        host: str,
        port: int,
    ) -> None:
        """Construct a new connection listener.

        Parameter
        ---------
        handler: Callable[[PConnection], None]
            Callback function for handling incoming connection requests.

        name: str
            A name describing the type of entity this connection listener serves (e.g.: agent, monitor, etc.).

        host: str
            The server host (IP) address.

        port: int
            The port on which to listen for incoming connections.
        """

        self._handler: Final[Callable[[PConnection], None]] = handler
        """Callback function for serving incoming connection requests."""

        self.name: Final[str] = name
        """A name describing the type of entity this connection listener serves (e.g.: agent, monitor, etc.)."""

        self.host: Final[str] = host
        """The server host (IP) address."""

        self.port: Final[int] = port
        """The port on which to listen for incoming connections."""

        self._sock: socket.socket | None = None
        """The socket for listening for incoming connections."""

        self._listener_thread: Thread | None = None
        """The thread used to listen for incoming connections."""

    def bind(self) -> None:
        """Bind to specified socket address."""

        if self._sock is not None:
            # socket already bound
            return

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.host, self.port))
            self._sock.listen(5)
        except ConnectionError:
            if self._sock is not None:
                self._sock.shutdown(socket.SHUT_RDWR)
                self._sock.close()
            raise

    def listen_for_connections(self) -> None:
        """Start listening for incoming connections."""

        if self.is_alive():
            # already listening for incoming connections
            return

        # ensure valid socket instance
        self.bind()

        # start listener thread
        self._listener_thread = Thread(target=self._listen_loop, name=f'{self.name}_connections_listener')
        self._listener_thread.start()

    def is_alive(self) -> bool:
        """Return whether the connection listener is alive."""

        return self._listener_thread is not None and self._listener_thread.is_alive()

    def join(self) -> None:
        """Wait until the listener thread terminates (if existing)."""

        if self._listener_thread is not None:
            self._listener_thread.join()

    def shutdown(self) -> None:
        """Shutdown connection listener."""

        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)

                # close and cleanup socket directly in case the listener thread has not been started, yet
                if self._listener_thread is None:
                    self._sock.close()
                    self._sock = None

            except Exception:  # noqa: BLE001
                logger.debug('ERROR while shutting down %s socket!', self.name, exc_info=True)

    def _listen_loop(self) -> None:
        """Wait for incoming connection requests."""

        if self._sock is None:
            return

        logger.info('Listening for %s connections on %s:%d', self.name, self.host, self.port)
        while True:
            try:
                sock, addr = self._sock.accept()
            except Exception:  # noqa: BLE001
                break

            # call connection handler in new thread
            conn_thread = Thread(target=self._serve, name='conn_handler', args=[sock, addr])
            conn_thread.start()

        logger.info('Shutdown %s connection listener thread.', self.name)
        self._sock.close()
        self._sock = None

    def _create_connection(self, sock: socket.socket, addr: socket.AddressInfo) -> PConnection:
        """Create a connection for the given socket and addr info."""

        return TCPLPMConnection(sock, addr)

    def _serve(self, sock: socket.socket, addr: socket.AddressInfo) -> None:
        """Serve connection."""

        # create connection wrapper
        conn = self._create_connection(sock, addr)
        logger.info('New %s connection: %s.', self.name, conn)

        # call external connection handler
        self._handler(conn)

        # shutdown and close connection after connection handler function finished
        conn.shutdown()
        conn.close()
