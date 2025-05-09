import argparse
import logging
import signal
from types import FrameType

from rcsssmj.game.field import SoccerFieldVersions, create_soccer_field
from rcsssmj.game.referee import SoccerReferee
from rcsssmj.game.rules import SoccerRuleBooks, create_soccer_rule_book
from rcsssmj.sim_server import SimServer

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

    rule_books = [str(book.value) for book in SoccerRuleBooks if book != SoccerRuleBooks.UNKNOWN]
    field_versions = [str(version.value) for version in SoccerFieldVersions if version != SoccerFieldVersions.UNKNOWN]

    # fmt: off
    # simulator arguments
    parser.add_argument('-a', '--host',     help='The server address.',                 default='127.0.0.1', type=str)
    parser.add_argument('-c', '--cport',    help='The client port.',                    default=60000,       type=int)
    parser.add_argument('-m', '--mport',    help='The monitor port.',                   default=60001,       type=int)
    parser.add_argument('-s', '--sync',     help='Run synchronous with agent clients.', default=False,       action='store_true')
    parser.add_argument('-r', '--realtime', help='Run in real-time mode.',              default=True,        action='store_true')
    parser.add_argument('-v', '--render',   help='Start internal monitor viewer.',      default=True,        action='store_true')

    # game arguments
    parser.add_argument('-f', '--field',    help='The soccer field version.',           default=SoccerFieldVersions.FIFA.value, type=str, choices=field_versions)
    parser.add_argument('-b', '--rules',    help='The soccer rule book.',               default=SoccerRuleBooks.FIFA.value, type=str, choices=rule_books)
    # fmt: on

    args = parser.parse_args()

    # create game referee
    soccer_field = create_soccer_field(args.field)
    rule_book = create_soccer_rule_book(args.rules, soccer_field)
    referee = SoccerReferee(rule_book)

    # create server
    server = SimServer(referee, args.host, args.cport, args.mport, sync_mode=args.sync, real_time=args.realtime, render=args.render)

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
