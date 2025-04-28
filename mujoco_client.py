import argparse
import signal
import socket
import threading
import time
from collections.abc import Mapping
from types import FrameType
from typing import ClassVar

import numpy as np


class Client:
    """
    Example client performing random actions.
    """

    _BEAM_POSES: ClassVar[Mapping[int, tuple[float, float, float]]] = {
        1: (29.0, 0.0, 0),
        2: (22.0, 12.0, 0),
        3: (22.0, 4.0, 0),
        4: (22.0, -4.0, 0),
        5: (22.0, -12.0, 0),
        6: (15.0, 0.0, 0),
        7: (4.0, 16.0, 0),
        8: (11.0, 6.0, 0),
        9: (11.0, -6.0, 0),
        10: (4.0, -16.0, 0),
        11: (7.0, 0.0, 0),
    }

    def __init__(self, host: str, port: int, team: str, player_no: int):
        """
        Construct a new agent connecting to the given server.
        """

        self._host: str = host
        self._port: int = port

        self._model_name: str = 'ant'
        self._team: str = team
        self._player_no: int = player_no

        self._rcv_buffer_size = 1024
        self._rcv_buffer = bytearray(self._rcv_buffer_size)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self._has_beamed: bool = False

        # set TCP_NODELAY option to send messages immediately (without buffering)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        print('[CONNECTED] Connected to the server.')

    def run(self):
        """
        Run the simulation client.
        """

        # connect to server
        self._sock.connect((self._host, self._port))

        # create client thread
        client_thread = threading.Thread(target=self._action_loop)
        client_thread.start()

        # wait for client thread to finish
        client_thread.join()

        # close server connection
        self._sock.close()

    def shutdown(self) -> None:
        """
        Shutdown the client.
        """

        self._sock.shutdown(socket.SHUT_RDWR)

    def _action_loop(self):
        """
        Main loop of the agent.
        """

        init_msg = f'(init {self._model_name} {self._team} {self._player_no})'
        self._send_message(init_msg.encode())

        print('[TEAM] Sent init message.')

        while True:
            try:
                # receive perception message
                perception_msg = self._receive_message()
                # print(perception_msg.decode())

                # perform action
                action = np.random.uniform(-1, 1, 8)  # random action
                # action = np.zeros(8)  # zero action
                action_msg = f'(l4e1 {action[0]:.2f})(l4e2 {action[1]:.2f})(l1e1 {action[2]:.2f})(l1e2 {action[3]:.2f})(l2e1 {action[4]:.2f})(l2e2 {action[5]:.2f})(l3e1 {action[6]:.2f})(l3e2 {action[7]:.2f})'

                if not self._has_beamed:
                    # action_msg += '(beam ' + ' '.join([str(val) for val in Client.BEAM_POSES[self.player_id]]) + ')'
                    beam_pose: tuple[float, float, float] = self._BEAM_POSES[self._player_no]
                    action_msg += f'(beam {beam_pose[0]} {beam_pose[1]} {beam_pose[2]})'
                    self._has_beamed = True

                # send action message
                self._send_message(action_msg.encode())
            except Exception:
                print('Connection closed.')
                break

    def _send_message(self, msg: bytes | bytearray) -> None:
        """
        Receive the next message from the TCP/IP socket.
        """

        self._sock.send((len(msg)).to_bytes(4, byteorder='big') + msg)

    def _receive_message(self) -> bytes | bytearray:
        """
        Receive the next message from the TCP/IP socket.
        """

        # receive message length information
        if self._sock.recv_into(self._rcv_buffer, nbytes=4, flags=socket.MSG_WAITALL) != 4:
            raise ConnectionResetError

        msg_size = int.from_bytes(self._rcv_buffer[:4], byteorder='big', signed=False)

        # ensure receive buffer is large enough to hold the message
        if msg_size > self._rcv_buffer_size:
            self._rcv_buffer_size = msg_size
            self._rcv_buffer = bytearray(self._rcv_buffer_size)

        # receive message with the specified length
        if self._sock.recv_into(self._rcv_buffer, nbytes=msg_size, flags=socket.MSG_WAITALL) != msg_size:
            raise ConnectionResetError

        return self._rcv_buffer[:msg_size]


if __name__ == '__main__':
    # parse arguments
    parser = argparse.ArgumentParser(description='The RocoCup MuJoCo Soccer Simulation Example Client.')

    # fmt: off
    parser.add_argument('-s', '--host',      type=str, help='The server address.', default='127.0.0.1', required=False)
    parser.add_argument('-p', '--port',      type=int, help='The server port.',    default=60000,       required=False)
    parser.add_argument('-t', '--team',      type=str, help='The team name.',      default='Test',      required=False)
    parser.add_argument('-n', '--player_no', type=int, help='The player number.',  default=1,           required=False)
    # fmt: on

    args = parser.parse_args()

    # create client
    client = Client(args.host, args.port, args.team, args.player_no)

    # register SIGINT handler
    def signal_handler(sig: int, frame: FrameType | int | signal.Handlers | None) -> None:
        del sig, frame  # signal unused parameter
        client.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # run client
    client.run()
