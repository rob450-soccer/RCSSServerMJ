import argparse
import logging
import signal
from types import FrameType

from rcsssmj.sim_server import Server

# ---------- LOGGING CONFIG ----------
# console handler
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
ch.setLevel(logging.INFO)

# file handler
fh = logging.FileHandler(filename='console.log', mode='w')
fh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s', '%Y-%m-%d %H:%M:%S'))
fh.setLevel(logging.DEBUG)

# configure logging
logging.basicConfig(handlers=[ch, fh], level=logging.DEBUG)
# ---------- LOGGING CONFIG ----------


logger = logging.getLogger(__name__)


def soccer_sim() -> None:
    """
    Main function for running the MuJoCo based Soccer Simulator.
    """

    # parse arguments
    parser = argparse.ArgumentParser(description='The RocoCup MuJoCo Soccer Simulation Server.')

    # fmt: off
    parser.add_argument('-a', '--host',     help='The server address.',                 default='127.0.0.1', type=str)
    parser.add_argument('-c', '--cport',    help='The client port.',                    default=60000,       type=int)
    parser.add_argument('-m', '--mport',    help='The monitor port.',                   default=60001,       type=int)
    parser.add_argument('-s', '--sync',     help='Run synchronous with agent clients.', default=False,       action='store_true')
    parser.add_argument('-r', '--realtime', help='Run in real-time mode.',              default=True,        action='store_true')
    parser.add_argument('-v', '--render',   help='Start internal monitor viewer.',      default=True,        action='store_true')
    # fmt: on

    args = parser.parse_args()

    # create server
    server = Server(args.host, args.cport, args.mport, sync_mode=args.sync, real_time=args.realtime, render=args.render)

    # register SIGINT handler
    def signal_handler(sig: int, frame: FrameType | int | signal.Handlers | None) -> None:
        del sig, frame  # signal unused parameter
        logger.debug(' --> HANDLE SIGINT <--')
        server.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # run server
    server.run()


if __name__ == '__main__':
    soccer_sim()
