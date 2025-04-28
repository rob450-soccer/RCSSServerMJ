import argparse
import signal
from types import FrameType

from rcsssmj.sim_server import Server


def soccer_sim() -> None:
    """
    Main function for running the MuJoCo based Soccer Simulator.
    """

    # parse arguments
    parser = argparse.ArgumentParser(description='The RocoCup MuJoCo Soccer Simulation Server.')

    # fmt: off
    parser.add_argument('-s', '--host',         type=str, help='The server address.', default='127.0.0.1', required=False)
    parser.add_argument('-p', '--port',         type=int, help='The client port.',    default=60000,       required=False)
    parser.add_argument('-m', '--monitor_port', type=int, help='The monitor port.',   default=60001,       required=False)
    # fmt: on

    args = parser.parse_args()

    # create server
    server = Server(args.host, args.port, args.monitor_port)

    # register SIGINT handler
    def signal_handler(sig: int, frame: FrameType | int | signal.Handlers | None) -> None:
        del sig, frame  # signal unused parameter
        print(' -> handle SIGINT...')
        server.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # run server
    server.run()


if __name__ == '__main__':
    soccer_sim()
