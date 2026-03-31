#!/usr/bin/env python3
"""
Simple monitor client for sending commands to the soccer simulation server.

Usage:
    python monitor_client.py kickOff Left    # Kick-off for Left team
    python monitor_client.py kickOff Right   # Kick-off for Right team
    python monitor_client.py kickOff          # Random kick-off
    python monitor_client.py dropBall         # Drop ball
"""

import argparse
import socket
import sys
import time


# Time to keep connection open so the server can collect and process the command
# before we disconnect (server runs at ~50 steps/s, so 0.5s gives plenty of margin).
_WAIT_AFTER_SEND_S = 0.5


def send_command(host: str, port: int, command: str) -> None:
    """Send a command to the monitor port."""
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    try:
        sock.connect((host, port))
        print(f'Connected to monitor at {host}:{port}')
        
        # Encode command as UTF-8
        msg_bytes = command.encode('utf-8')
        
        # Send length-prefixed message (4-byte big-endian length + message)
        length_prefix = len(msg_bytes).to_bytes(4, byteorder='big')
        sock.send(length_prefix + msg_bytes)
        
        print(f'Sent command: {command}')
        
        # Keep connection open so the server can process the command before we
        # disconnect (otherwise the monitor is removed and the command is never collected).
        time.sleep(_WAIT_AFTER_SEND_S)
        
    except ConnectionRefusedError:
        print(f'Error: Connection refused. Make sure the server is running on {host}:{port}', file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description='Send commands to the soccer simulation monitor.')
    parser.add_argument('-s', '--host', type=str, help='Server host address', default='127.0.0.1')
    parser.add_argument('-p', '--port', type=int, help='Monitor port', default=60001)
    parser.add_argument('command', type=str, nargs='+', help='Command to send (e.g., "kickOff Left" or "dropBall")')
    
    args = parser.parse_args()
    
    # Join command parts into a single S-expression string
    command_str = '(' + ' '.join(args.command) + ')'
    
    send_command(args.host, args.port, command_str)


if __name__ == '__main__':
    main()