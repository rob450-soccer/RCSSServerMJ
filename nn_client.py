import argparse
import logging
import signal
import socket
import threading
from collections.abc import Mapping
from types import FrameType
from typing import ClassVar

import numpy as np
import re
from scipy.spatial.transform import Rotation as R
import torch
import torch.nn as nn
import torch.nn.functional as F

# ---------- LOGGING CONFIG ----------
# console handler
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
ch.setLevel(logging.INFO)

# configure logging
logging.basicConfig(handlers=[ch], level=logging.DEBUG)
# ---------- LOGGING CONFIG ----------


logger = logging.getLogger(__name__)


class Client:
    """
    Example client performing random actions.
    """

    BEAM_POSES: ClassVar[Mapping[int, tuple[float, float, float]]] = {
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

    ROBOT_MOTORS: ClassVar[Mapping[str, tuple[str, ...]]] = {
        'ant': ('l4e1', 'l4e2', 'l1e1', 'l1e2', 'l2e1', 'l2e2', 'l3e1', 'l3e2'),
        'T1': ('he1', 'he2', 'lae1', 'lae2', 'lae3', 'lae4', 'rae1', 'rae2', 'rae3', 'rae4', 'te1', 'lle1', 'lle2', 'lle3', 'lle4', 'lle5', 'lle6', 'rle1', 'rle2', 'rle3', 'rle4', 'rle5', 'rle6'),
    }

    def __init__(self, host: str, port: int, team: str, player_no: int, model_name: str | None = None):
        """
        Construct a new agent connecting to the given server.
        """

        self._host: str = host
        self._port: int = port

        self._model_name: str = 'ant' if model_name is None else model_name
        self._team: str = team
        self._player_no: int = player_no

        self._rcv_buffer_size = 1024
        self._rcv_buffer = bytearray(self._rcv_buffer_size)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self._has_beamed: bool = False

        # set TCP_NODELAY option to send messages immediately (without buffering)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def run(self):
        """
        Run the simulation client.
        """

        # connect to server
        logger.info('Connecting to server at %s:%d...', self._host, self._port)
        try:
            self._sock.connect((self._host, self._port))
        except ConnectionRefusedError:
            logger.error('Connection refused. Make sure the server is running and listening on the specified interface.')  # noqa: TRY400
            return
        # logger.info('Server connection established.')

        # create client thread
        client_thread = threading.Thread(target=self._action_loop)
        client_thread.start()

        # wait for client thread to finish
        client_thread.join()

        # logger.info('Shutting down.')

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

        logger.info('Initializing agent...')
        init_msg = f'(init {self._model_name} {self._team} {self._player_no})'
        self._send_message(init_msg.encode())
        # logger.info('Initialization complete.')

        self.nr_joints = len(self.ROBOT_MOTORS[self._model_name])
        self.previous_action = np.zeros(self.nr_joints)
        self.p_gain = 25.0
        self.d_gain = 0.6
        self.scaling_factor = 0.5
        self.joint_nominal_position = np.array([
            0.0, 0.0,
            0.0, 1.4, 0.0, -0.4,
            0.0, -1.4, 0.0, 0.4,
            0.0,
            -0.4, 0.0, 0.0, 0.8, -0.4, 0.0,
            0.4, 0.0, 0.0, -0.8, 0.4, 0.0,
        ])
        train_sim_flip = np.array([
            1.0, -1.0,
            1.0, -1.0, -1.0, 1.0,
            -1.0, -1.0, 1.0, 1.0,
            1.0,
            1.0, -1.0, -1.0, 1.0, 1.0, -1.0,
            -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
        ])
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = PolicyNetwork().to(self.device)
        self.policy.load_state_dict(torch.load("locomotion_nn.pth", map_location=self.device))

        wait_until_walking = 50

        logger.info('Running perception-action-loop.')
        while True:
            try:
                # receive perception message
                perception_msg = self._receive_message()
                perception_msg_str = perception_msg.decode()
                perception_data = self.parse_sensor_string(perception_msg_str)

                joint_pos_degrees = np.array([h["ax"] for h in perception_data["HJ"]])
                joint_pos = np.deg2rad(joint_pos_degrees)

                joint_vel_degrees = np.array([h["vx"] for h in perception_data["HJ"]])
                joint_vel = np.deg2rad(joint_vel_degrees)

                qpos_qvel_previous_action = np.vstack(([
                    ((joint_pos * train_sim_flip) - self.joint_nominal_position) / 4.6,
                    joint_vel / 110.0 * train_sim_flip,
                    self.previous_action / 10.0,
                ])).T.flatten()
                
                ang_vel = np.clip(np.deg2rad(perception_data["GYR"]["rt"]) / 50.0, -1.0, 1.0)
                orientation_quat_mj_convention = perception_data["quat"]["q"]
                orientation_quat_inv = R.from_quat([
                    orientation_quat_mj_convention[1],
                    orientation_quat_mj_convention[2],
                    orientation_quat_mj_convention[3],
                    orientation_quat_mj_convention[0],
                ]).inv()
                projected_gravity = orientation_quat_inv.apply(np.array([0.0, 0.0, -1.0]))

                wait_until_walking  = max(0, wait_until_walking - 1)
                if wait_until_walking > 0:
                    goal_vel = np.array([0.0, 0.0, 0.0])
                else:
                    goal_vel = np.array([0.5, 0.0, 0.0])

                observation = np.concatenate([
                    qpos_qvel_previous_action,
                    ang_vel,
                    goal_vel,
                    projected_gravity,
                ])
                observation = np.clip(observation, -10.0, 10.0)

                nn_action = self.policy(torch.tensor(observation, dtype=torch.float32).to(self.device)).detach().cpu().numpy()

                target_joint_positions = self.joint_nominal_position + self.scaling_factor * nn_action
                target_joint_positions *= train_sim_flip

                msg_list: list[str] = []
                motors = self.ROBOT_MOTORS[self._model_name]
                for motor, target_joint_position in zip(motors, target_joint_positions, strict=False):
                    msg_list.append(f'({motor} {target_joint_position:.2f} 0.0 {self.p_gain:.2f} {self.d_gain:.2f} 0.0)')

                if not self._has_beamed:
                    # msg_list.append('(beam ' + ' '.join([str(val) for val in self.BEAM_POSES[self.player_id]]) + ')')
                    beam_pose = self.BEAM_POSES[self._player_no]
                    msg_list.append(f'(beam {beam_pose[0]} {beam_pose[1]} {beam_pose[2]})')
                    self._has_beamed = True

                self.previous_action = nn_action

                # send action message
                action_msg = ''.join(msg_list)
                self._send_message(action_msg.encode())
            except Exception:
                logger.info('Server connection closed.')
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


    def parse_sensor_string(self, s: str) -> dict:
        """
        Parses a sensor data string of nested parenthesis groups into a structured dictionary.
        Repeated top-level tags are aggregated into lists.
        """
        result = {}
        # Top-level groups: (TAG ...content...)
        top_level_pattern = re.compile(r'\((\w+)((?:\s*\([^()]*\))*)\)')
        
        for tag, inner in top_level_pattern.findall(s):
            # Find inner key-value or key-list groups: (key val1 val2 ...)
            items = re.findall(r'\(\s*(\w+)((?:\s+[^()]+)+)\)', inner)
            group = {}
            for key, vals in items:
                tokens = vals.strip().split()
                parsed_vals = []
                for t in tokens:
                    try:
                        parsed_vals.append(float(t))
                    except ValueError:
                        parsed_vals.append(t)
                # Single value vs. list
                group[key] = parsed_vals[0] if len(parsed_vals) == 1 else parsed_vals
            
            # Merge into result, handling repeated tags as lists
            if tag in result:
                if isinstance(result[tag], list):
                    result[tag].append(group)
                else:
                    result[tag] = [result[tag], group]
            else:
                result[tag] = group
        
        return result


class PolicyNetwork(nn.Module):
    def __init__(self):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(78, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc4 = nn.Linear(128, 23)
        self.layer_norm = nn.LayerNorm(512, eps=1e-6)

    def forward(self, x):
        x = F.elu(self.layer_norm(self.fc1(x)))
        x = F.elu(self.fc2(x))
        x = F.elu(self.fc3(x))
        x = self.fc4(x)
        x = torch.clip(x, -10.0, 10.0)
        return x


if __name__ == '__main__':
    # parse arguments
    parser = argparse.ArgumentParser(description='The RocoCup MuJoCo Soccer Simulation Example Client.')

    robots = list(Client.ROBOT_MOTORS.keys())

    # fmt: off
    parser.add_argument('-s', '--host',      type=str, help='The server address.', default='127.0.0.1', required=False)
    parser.add_argument('-p', '--port',      type=int, help='The server port.',    default=60000,       required=False)
    parser.add_argument('-t', '--team',      type=str, help='The team name.',      default='Test',      required=False)
    parser.add_argument('-n', '--player_no', type=int, help='The player number.',  default=1,           required=False)
    parser.add_argument('-r', '--robot',     type=str, help='The robot model.',    default=robots[0],   required=False, choices=robots)
    # fmt: on

    args = parser.parse_args()

    # create client
    client = Client(args.host, args.port, args.team, args.player_no, args.robot)

    # register SIGINT handler
    def signal_handler(sig: int, frame: FrameType | int | signal.Handlers | None) -> None:
        del sig, frame  # signal unused parameter
        client.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # run client
    client.run()
