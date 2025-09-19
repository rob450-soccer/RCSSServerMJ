import logging
from abc import ABC, abstractmethod
from collections.abc import Sequence
from queue import Empty
from threading import Lock
from typing import Any, Final, cast

import mujoco
import numpy as np

from rcsssmj.agent import AgentID, PAgent
from rcsssmj.client.action import SimAction
from rcsssmj.client.encoder import DefaultPerceptionEncoder, PerceptionEncoder
from rcsssmj.client.parser import ActionParser, DefaultActionParser
from rcsssmj.client.perception import (
    AccelerometerPerception,
    AgentDetection,
    GyroPerception,
    JointStatePerception,
    ObjectDetection,
    OrientationPerception,
    Perception,
    PObjectDetection,
    PositionPerception,
    TimePerception,
    TouchPerception,
    VisionPerception,
)
from rcsssmj.client.sim_client import RemoteSimClient, SimClient, SimClientState
from rcsssmj.communication.connection import PConnection
from rcsssmj.monitor.commands import MonitorCommand
from rcsssmj.monitor.parser import CommandParser, DefaultCommandParser
from rcsssmj.monitor.sim_monitor import RemoteSimMonitor, SimMonitor, SimMonitorState
from rcsssmj.monitor.state import SceneGraph, SimStateInformation
from rcsssmj.resources.spec_provider import ModelSpecProvider

logger = logging.getLogger(__name__)


class BaseSimulation(ABC):
    """Base class for simulations."""

    def __init__(
        self,
        *,
        action_parser: ActionParser | None = None,
        perception_encoder: PerceptionEncoder | None = None,
        command_parser: CommandParser | None = None,
        spec_provider: ModelSpecProvider | None = None,
        n_substeps: int = 4,
        vision_interval: int = 1,
    ) -> None:
        """Construct a new simulation.

        Parameter
        ---------
        action_parser: ActionParser | None, default=None
            Parser instance for parsing remote client actions.

        perception_encoder: PerceptionEncoder | None, default=None
            Encoder instance for encoding perceptions for remote clients.

        command_parser: CommandParser | None, default=None
            Parser instance for parsing remote monitor commands.

        spec_provider: ModelSpecProvider | None, default=None
            MuJoCo model specification provider instance to use for loading model specifications.

        substeps: int, default=4
            The number of simulation substeps between the agent update cycles.

        vision_interval: int, default=1
            The interval in which vision perception will be generated.
        """

        self.action_parser: Final[ActionParser] = DefaultActionParser() if action_parser is None else action_parser
        """Parser for parsing client action messages."""

        self.perception_encoder: Final[PerceptionEncoder] = DefaultPerceptionEncoder() if perception_encoder is None else perception_encoder
        """Encoder for encoding client perception messages."""

        self.command_parser: Final[CommandParser] = DefaultCommandParser() if command_parser is None else command_parser
        """Parser for parsing monitor command messages."""

        self.spec_provider: Final[ModelSpecProvider] = ModelSpecProvider() if spec_provider is None else spec_provider
        """Mujoco model specification provider for loading models."""

        self.n_substeps: Final[int] = n_substeps
        """The number of simulation substeps."""

        self.vision_interval: Final[int] = vision_interval
        """The interval (in simulation cycles) in which the vision perception is generated."""

        self._frame_id: int = 0
        """The current simulation frame number."""

        self._mj_spec: Any = None
        """The mujoco simulation model specification."""

        self._mj_model: Any = None
        """The mujoco simulation model."""

        self._mj_data: Any = None
        """The mujoco simulation data array."""

        self._world_markers: Sequence[tuple[str, str]] = []
        """The sequence of world markers used for generating vision perceptions."""

        self._clients: list[SimClient] = []
        """The list of connected clients."""

        self._monitors: list[SimMonitor] = []
        """The list of connected monitors."""

        self._mutex: Lock = Lock()
        """Mutex for synchronizing simulation threads."""

    @property
    def frame_id(self) -> int:
        """The simulation frame id."""

        return self._frame_id

    @property
    def mj_spec(self) -> Any:
        """The mujoco simulation model specification."""

        return self._mj_spec

    @property
    def mj_model(self) -> Any:
        """The mujoco simulation model."""

        return self._mj_model

    @property
    def mj_data(self) -> Any:
        """The mujoco simulation data array."""

        return self._mj_data

    @property
    def sim_time(self) -> float:
        """The current simulation time."""

        return 0.0 if self._mj_data is None else self._mj_data.time

    def kill_sim(self) -> None:
        """Kill the simulation (server)."""

        # TODO: Implement some sort of kill-flag that can be evaluated in external components (mainly the sim server) to trigger a shutdown.

    def register_clients(self, *client_or_conns: SimClient | PConnection) -> None:
        """Register new clients with the simulation.

        Parameter
        ---------
        *client_or_conns: SimClient | PConnection
            The client instances to add / connections for which to add remote clients.
        """

        # create remote client instances in case the parameter contain connections
        clients = [client_or_conn if isinstance(client_or_conn, SimClient) else RemoteSimClient(client_or_conn, self.action_parser, self.perception_encoder) for client_or_conn in client_or_conns]

        with self._mutex:
            self._clients.extend(clients)

    def register_monitors(self, *monitor_or_conns: SimMonitor | PConnection) -> None:
        """Register new monitors with the simulation.

        Parameter
        ---------
        *monitor_or_conns: SimMonitor | PConnection
            The monitor instances to add / connections for which to add remote monitors.
        """

        # create remote monitor instances in case the parameter contain connections
        monitors = [monitor_or_conn if isinstance(monitor_or_conn, SimMonitor) else RemoteSimMonitor(monitor_or_conn, self.command_parser) for monitor_or_conn in monitor_or_conns]

        with self._mutex:
            self._monitors.extend(monitors)

    def remove_clients(self, *clients: SimClient) -> None:
        """Remove the given clients from the simulation.

        Note:
        This method will not automatically deactivate the given clients.
        Make sure to deactivate the client instances before calling this method.

        Parameter
        ---------
        *clients: SimClient
            The client instances to remove.
        """

        with self._mutex:
            for client in clients:
                self._clients.remove(client)

    def remove_monitors(self, *monitors: SimMonitor) -> None:
        """Remove the given monitors from the simulation.

        Note:
        This method will not automatically deactivate the given monitors.
        Make sure to shutdown the monitor instances before calling this method.

        Parameter
        ---------
        *monitors: SimMonitor
            The monitor instances to remove.
        """

        with self._mutex:
            for monitor in monitors:
                self._monitors.remove(monitor)

    def filter_clients(self) -> tuple[list[SimClient], list[SimClient], list[SimClient], list[SimClient]]:
        """Filter simulation clients by state.

        Returns
        -------
        connected_clients: list[SimClient]
            The list of clients in connected state.

        ready_clients: list[SimClient]
            The list of clients in ready state.

        active_clients: list[SimClient]
            The list of clients in active state.

        disconnected_clients: list[SimClient]
            The list of clients in disconnected state.
        """

        connected_clients: list[SimClient] = []
        ready_clients: list[SimClient] = []
        active_clients: list[SimClient] = []
        disconnected_clients: list[SimClient] = []

        with self._mutex:
            for client in self._clients:
                state = client.get_state()
                if state == SimClientState.INIT:
                    connected_clients.append(client)
                elif state == SimClientState.READY:
                    ready_clients.append(client)
                elif state == SimClientState.ACTIVE:
                    active_clients.append(client)
                # elif state == SimClientState.DISCONNECTED:
                else:
                    disconnected_clients.append(client)

        return connected_clients, ready_clients, active_clients, disconnected_clients

    def filter_monitors(self) -> tuple[list[SimMonitor], list[SimMonitor]]:
        """Filter simulation monitors by state.

        Returns
        -------
        active_monitors: list[SimMonitor]
            The list of active / connected monitors.

        inactive_monitors: list[SimMonitor]
            The list of inactive / disconnected monitors.
        """

        active_monitors: list[SimMonitor] = []
        inactive_monitors: list[SimMonitor] = []

        with self._mutex:
            for monitor in self._monitors:
                if monitor.get_state() == SimMonitorState.ACTIVE:
                    active_monitors.append(monitor)
                else:
                    inactive_monitors.append(monitor)

        return active_monitors, inactive_monitors

    def init(self) -> bool:
        """Initialize the game and create a new simulation world environment."""

        # clear existing clients in case there are any to prevent memory leaks
        with self._mutex:
            for client in self._clients:
                client.shutdown()
            self._clients.clear()

        # initialize game and create game world environment
        self._mj_spec = self._create_world()
        if self._mj_spec is None:
            logger.warning('Failed to initialize game.')
            return False

        # prepare initial simulation model and data
        self._mj_model = self._mj_spec.compile()
        self._mj_data = mujoco.MjData(self._mj_model)

        # extract visible object markers of world
        self._world_markers = [(site.name, site.name[:-10]) for site in self._mj_spec.sites if site.name.endswith('-vismarker')]

        # reset frame id
        self._frame_id = 0

        return True

    def shutdown(self, *, wait: bool = False) -> None:
        """Shutdown simulation."""

        # shutdown active clients
        for client in self._clients:
            client.shutdown(wait=wait)
        self._clients.clear()

        # shutdown active monitors
        for monitor in self._monitors:
            monitor.shutdown(wait=wait)
        self._monitors.clear()

        self._mj_spec = None
        self._mj_model = None
        self._mj_data = None
        self._world_markers = []

    def activate_clients(self, ready_clients: Sequence[SimClient]) -> tuple[list[SimClient], list[SimClient]]:
        """Try activate the given list of clients.

        Parameter
        ---------
        ready_clients: Sequence[SimClient]
            The list of clients to activate.
        """

        activated_clients: list[SimClient] = []
        clients_to_remove: list[SimClient] = []

        for client in ready_clients:
            if self._activate_client(client):
                activated_clients.append(client)
            else:
                # failed to activate client --> shutdown and remove client
                logger.info('Failed to activate client %s. Shutting down client again.', client)
                client.shutdown()
                clients_to_remove.append(client)

        if activated_clients:
            # recompile spec in case new clients got added
            self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)

            # calculate forward kinematics / dynamics of newly added robot models (without progressing the time)
            mujoco.mj_forward(self._mj_model, self._mj_data)

        return activated_clients, clients_to_remove

    def _activate_client(self, client: SimClient) -> bool:
        """Try to activate the given client.

        Parameter
        ---------
        client: SimClient
            The simulation client to activate.
        """

        # try to load the robot model requested by the client
        robot_spec = self.spec_provider.load_robot(client.get_model_name())
        if robot_spec is None:
            # failed to load the requested model --> report failure
            return False

        # request participation in game
        agent_id = self._request_participation(client, robot_spec)
        if agent_id is None:
            # invalid team side -> report failure
            return False

        # append robot to world
        frame = self._mj_spec.worldbody.add_frame()
        frame.attach_body(robot_spec.body('torso'), agent_id.prefix, '')

        client.activate(agent_id, robot_spec)

        logger.info('Player %s joined the game.', client)

        return True

    def deactivate_clients(self, clients: Sequence[SimClient]) -> None:
        """Deactivate the given list of clients.

        Parameter
        ---------
        clients: Sequence[SimClient]
            The list of client instances to deactivate.
        """

        recompile_spec = False

        for client in clients:
            recompile_spec |= self._deactivate_client(client)

        if recompile_spec:
            self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)

    def _deactivate_client(self, client: SimClient) -> bool:
        """Deactivate the given client instance.

        Parameter
        ---------
        client: SimClient
            The simulation client to activate.
        """

        # check if client has been activated before
        agent_id = client.get_id()

        if agent_id is not None:
            # remove client model from simulation
            self._mj_spec.delete(self._mj_spec.body(agent_id.prefix + 'torso'))

            # delete various components manually, as they are not automatically removed again when the root body is detached
            # Note:
            # Not sure if this is intentional behavior or a bug in mujoco.
            # It's also not clear what components need to be deleted separately.
            # The code below so far prevents any follow-up exceptions when re-attaching the same model again.
            # But at the moment, there is no guarantee that there will be no components left in the spec that may cause some trouble at some point.
            def del_els(el_list: list[Any]) -> None:
                for el in el_list:
                    self._mj_spec.delete(el)

            model_spec = cast(Any, client.get_model_spec())
            # del_els(model_spec.cameras)
            # del_els(model_spec.geoms)
            # del_els(model_spec.lights)
            del_els(model_spec.materials)
            del_els(model_spec.meshes)
            del_els(model_spec.sites)
            del_els(model_spec.texts)
            del_els(model_spec.textures)

            # remove agent from game
            self._handle_withdrawal(agent_id)

            logger.info('Player %s left the game.', client)

            return True

        return False

    def collect_actions(self, active_clients: Sequence[SimClient], *, block: bool = False, timeout: float = 5) -> list[SimAction]:
        """Collect the actions from all active clients.

        Parameter
        ---------
        active_clients: Sequence[SimClient]
            The list of active clients.

        block: bool, default=False
            Wait for client actions to arrive.

        timeout: float, default=5
            The time to wait for client actions to arrive. After this time, the client is considered inactive and will be shutdown.
            If timeout is a negative number, it will wait forever.
        """

        client_actions: list[SimAction] = []

        # collect client actions and send perceptions
        for client in active_clients:
            # collect all pending actions
            action_queue = client.get_action_queue()
            try:
                if block:
                    # wait for exactly one client action
                    client_actions += action_queue.get(timeout=timeout)
                else:
                    # fetch all currently available actions
                    while action_queue.qsize() > 0:
                        client_actions += action_queue.get_nowait()
            except Empty:
                if block:
                    # client took too long to answer -> kill it
                    logger.info('Team %s: Player %d did not respond for more than %.3f seconds. Forcing player shutdown.', client.get_team_name(), client.get_player_no(), timeout)
                    client.shutdown()
                    continue

        return client_actions

    def collect_commands(self, active_monitors: Sequence[SimMonitor]) -> list[MonitorCommand]:
        """Collect the commands from all active monitors.

        Parameter
        ---------
        active_monitors: Sequence[SimMonitor]
            The list of active monitors.
        """

        monitor_commands: list[MonitorCommand] = []

        for monitor in active_monitors:
            command_queue = monitor.get_command_queue()
            try:
                while command_queue.qsize() > 0:
                    monitor_commands.append(command_queue.get_nowait())
            except Empty:
                pass

        return monitor_commands

    def step(self, client_actions: Sequence[SimAction], monitor_commands: Sequence[MonitorCommand]) -> None:
        """Perform a simulation step.

        Parameter
        ---------
        client_actions: Sequence[SimAction]
            The list of simulation actions.

        monitor_commands: Sequence[MonitorCommand]
            The list of monitor commands.
        """

        # pre-step hook
        self._pre_step(client_actions)

        # progress simulation
        mujoco.mj_step(self._mj_model, self._mj_data, self.n_substeps)

        # post-step hook
        self._post_step(monitor_commands)

        # increment frame id
        self._frame_id += 1

    def _pre_step(self, client_actions: Sequence[SimAction]) -> None:
        """Method executed right before a simulation step.

        Parameter
        ---------
        client_actions: Sequence[SimAction]
            The list of simulation actions.
        """

        # apply client actions
        for action in client_actions:
            action.perform(self)

    def _post_step(self, monitor_commands: Sequence[MonitorCommand]) -> None:
        """Method executed right after a simulation step.

        Parameter
        ---------
        monitor_commands: Sequence[MonitorCommand]
            The list of monitor commands.
        """

        # apply monitor commands
        for command in monitor_commands:
            command.perform(self)

    def generate_perceptions(self, active_clients: Sequence[SimClient], *, gen_vision: bool | None = None) -> None:
        """Generate perceptions for active clients.

        Parameter
        ---------
        active_clients: Sequence[SimClient]
            The list of clients considered as active in this simulation cycle.

        gen_vision: bool, default=None
            Generate vision perception. If None, vision is generated according to the `vision_interval` attribute.
        """

        def trunc2(val: float) -> float:
            """Limit the given value to two digits."""
            return int(val * 100) / 100.0

        def trunc2_vec(vec: Any) -> Any:
            """Limit the given vector to two digits."""
            return np.trunc(vec * 100) / 100.0

        def trunc3_vec(vec: Any) -> Any:
            """Limit the given vector to three digits."""
            return np.trunc(vec * 1000) / 1000.0

        # default to vision interval if gen_vision parameter is not specified
        if gen_vision is None:
            gen_vision = self._frame_id % self.vision_interval == 0

        # generate general perceptions equal for all clients
        sim_time_perception = TimePerception('now', trunc2(self._mj_data.time))
        game_state_perception = self._generate_game_state_perception()

        if gen_vision:
            # collect visible markers
            n_world_markers = len(self._world_markers)
            obj_markers = list(self._world_markers)
            for client in active_clients:
                obj_markers.extend(client.get_model_markers())

            # extract visible object positions
            n_markers = len(obj_markers)
            obj_pos = np.zeros((3, n_markers), dtype=np.float64)
            for idx, site in enumerate(obj_markers):
                obj_pos[:, idx] = self._mj_data.site(site[0]).xpos.astype(np.float64)

        # generate client specific perceptions
        for client in active_clients:
            joint_names: list[str] = []
            joint_axs: list[float] = []
            joint_vxs: list[float] = []
            client_perceptions: list[Perception] = [sim_time_perception, game_state_perception]

            model_spec = cast(Any, client.get_model_spec())
            agent_id = cast(AgentID, client.get_id())
            prefix_length = len(agent_id.prefix)

            for sensor_spec in model_spec.sensors:
                sensor = self._mj_data.sensor(sensor_spec.name)
                sensor_name = sensor_spec.name[prefix_length:]

                if sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    joint_names.append(sensor_name)
                    joint_axs.append(sensor.data[0])

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTVEL:
                    joint_vxs.append(sensor.data[0])

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_GYRO:
                    rvx, rvy, rvz = trunc2_vec(np.degrees(sensor.data[0:3]))
                    client_perceptions.append(GyroPerception(sensor_name, rvx, rvy, rvz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                    ax, ay, az = trunc2_vec(sensor.data[0:3])
                    client_perceptions.append(AccelerometerPerception(sensor_name, ax, ay, az))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_TOUCH:
                    active = int(sensor.data[0])
                    client_perceptions.append(TouchPerception(sensor_name, active))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEQUAT:
                    qw, qx, qy, qz = trunc3_vec(sensor.data[0:4])
                    client_perceptions.append(OrientationPerception(sensor_name, qw, qx, qy, qz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEPOS:
                    px, py, pz = trunc3_vec(sensor.data[0:3])
                    client_perceptions.append(PositionPerception(sensor_name, px, py, pz))

                # TODO: Add perceptions for force and hear

                else:
                    # sensor not supported...
                    pass

            # joint state perception
            if joint_names:
                client_perceptions.append(JointStatePerception(joint_names, trunc2_vec(np.degrees(joint_axs)), trunc2_vec(np.degrees(joint_vxs))))

            # ideal camera sensor-pipeline
            if gen_vision:
                # fetch camera sensor site
                camera_site = self._mj_data.site(agent_id.prefix + 'camera')

                if camera_site is not None:
                    # fetch pose of camera site of robot model
                    camera_pos = camera_site.xpos.astype(np.float64)
                    camera_rot = camera_site.xmat.astype(np.float64).reshape((3, 3))

                    # transform detectable obj positions to camera frame
                    local_obj_pos = np.matmul(camera_rot.T, obj_pos - camera_pos[:, np.newaxis])

                    # transform local positions into polar coordinates
                    azimuths = trunc2_vec(np.degrees(np.atan2(local_obj_pos[1], local_obj_pos[0])))
                    distances = np.linalg.norm(local_obj_pos, axis=0)
                    elevations = trunc2_vec(np.degrees(np.asin(local_obj_pos[2] / distances)))
                    distances = trunc2_vec(distances)

                    # TODO: Apply sensor noise

                    # check object coordinates for horizontal and vertical view range
                    half_horizontal_range = 60
                    half_vertical_range = 60
                    obj_visibility = (azimuths >= -half_horizontal_range) & (azimuths <= half_horizontal_range) & (elevations >= -half_vertical_range) & (elevations <= half_vertical_range)

                    # extract simple world object detections
                    obj_detections: list[PObjectDetection] = [ObjectDetection(obj_markers[i][1], azimuths[i], elevations[i], distances[i]) for i in range(n_world_markers) if obj_visibility[i]]

                    # extract agent object detections
                    idx = n_world_markers
                    for agent in active_clients:
                        n_agent_markers = len(agent.get_model_markers())
                        agent_detections = [ObjectDetection(obj_markers[i][1], azimuths[i], elevations[i], distances[i]) for i in range(idx, idx + n_agent_markers) if obj_visibility[i]]
                        if agent_detections:
                            obj_detections.append(AgentDetection('P', agent.get_team_name(), agent.get_player_no(), agent_detections))

                        idx += n_agent_markers

                    client_perceptions.append(VisionPerception('See', obj_detections))

            # forward generated perceptions to client instance
            client.set_perceptions(client_perceptions)

    def send_perceptions(self, active_clients: Sequence[SimClient]) -> None:
        """Send the previously generated perceptions to active clients.

        Parameter
        ---------
        active_clients: Sequence[SimClient]
            The list of active clients.
        """

        for client in active_clients:
            client.send_perceptions()

    def _generate_state_information(self) -> list[SimStateInformation]:
        """Generate simulation state information for updating monitor instances."""

        return [SceneGraph(self._mj_model, self._mj_data)]

    def update_monitors(self, active_monitors: Sequence[SimMonitor]) -> None:
        """Update active monitors.

        Parameter
        ---------
        active_monitors: Sequence[SimMonitor]
            The list of active monitors.

        referee: SoccerReferee
            The referee instance.
        """

        state_info = self._generate_state_information()

        for monitor in active_monitors:
            monitor.update(state_info, self._frame_id)

    @abstractmethod
    def _create_world(self) -> Any | None:
        """Create a new simulation world environment.

        Returns
        -------
        MjSpec
            The game specific simulation world / environment specification.
        """

    @abstractmethod
    def _request_participation(self, agent: PAgent, model_spec: Any) -> AgentID | None:
        """Validate and add a new agent requesting to join the game.

        Parameter
        ---------
        agent: PAgent
            The agent the requests participation in the game.

        model_spec: MjSpec
            The robot model specification of the new agent.

        Returns
        -------
        AgentID | None
            The agent id identifying the new agent if participation has been accepted, None if participation of the new agent has been rejected.
        """

    @abstractmethod
    def _handle_withdrawal(self, agent_id: AgentID) -> None:
        """Handle the withdrawal of an agent participating in the game.

        Parameter
        ---------
        aid: AgentID
            The id of the agent that withdrawed from the game.
        """

    @abstractmethod
    def _generate_game_state_perception(self) -> Perception:
        """Generate a perception representing the current game state to participating players.

        Returns
        -------
        Perception
            The game state perception.
        """
