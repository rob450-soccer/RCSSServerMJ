import logging
from collections.abc import Callable
from threading import Thread
from typing import Final

from websockets.sync.server import Server, ServerConnection, serve

from rcsssmj.server.communication.connection import PConnection
from rcsssmj.server.communication.ws_connection import WSConnection

logger = logging.getLogger(__name__)


class WSConnectionListener:
    """Helper class listening for incoming websocket connections on a specified interface and port.

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
        """Callback function for handling incoming connection requests."""

        self.name: Final[str] = name
        """A name describing the type of entity this connection listener serves (e.g.: agent, monitor, etc.)."""

        self.host: Final[str] = host
        """The server host (IP) address."""

        self.port: Final[int] = port
        """The port on which to listen for incoming connections."""

        self._wss: Server | None = None
        """The socket for listening for incoming connections."""

        self._listener_thread: Thread | None = None
        """The thread used to listen for incoming connections."""

    def bind(self) -> None:
        """Bind to specified socket address."""

        if self._wss is not None:
            # websocket already bound
            return

        try:
            self._wss = serve(self._serve, self.host, self.port)
        except ConnectionError:
            if self._wss is not None:
                self._wss.shutdown()
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

        if self._wss is not None:
            try:
                self._wss.shutdown()

                # close and cleanup socket directly in case the listener thread has not been started, yet
                if self._listener_thread is None:
                    self._sock = None

            except Exception:  # noqa: BLE001
                logger.debug('ERROR while shutting down %s websocket!', self.name, exc_info=True)

    def _listen_loop(self) -> None:
        """Wait for incoming connection requests."""

        if self._wss is None:
            return

        logger.info('Listening for %s connections on %s:%d', self.name, self.host, self.port)
        self._wss.serve_forever()

        logger.info('Shutdown %s connection listener thread.', self.name)
        self._wss = None

    def _serve(self, sc: ServerConnection) -> None:
        """Handle an incoming websocket connection request."""

        # create connection wrapper
        conn = WSConnection(sc)
        logger.info('New %s connection: %s.', self.name, conn)

        # call external connection handler
        self._handler(conn)

        # Note: Connection cleanup is handled automatically by the websockets server after this method finishes.
