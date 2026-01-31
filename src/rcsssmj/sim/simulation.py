import logging
from abc import ABC, abstractmethod
from collections.abc import Iterator, Sequence
from typing import Any, Final

import mujoco
import numpy as np

from rcsssmj.resources.spec_provider import ModelSpecProvider
from rcsssmj.sim.actions import SimAction
from rcsssmj.sim.agent_id import AgentID
from rcsssmj.sim.agent_params import PAgentParameter
from rcsssmj.sim.commands import MonitorCommand
from rcsssmj.sim.perceptions import (
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
from rcsssmj.sim.sim_agent import SimAgent
from rcsssmj.sim.sim_object import SimObject
from rcsssmj.sim.state_info import SceneGraph, SimStateInformation

logger = logging.getLogger(__name__)


class BaseSimulation(ABC):
    """Base class for simulations."""

    def __init__(
        self,
        *,
        spec_provider: ModelSpecProvider | None = None,
        n_substeps: int = 4,
        vision_interval: int = 1,
    ) -> None:
        """Construct a new simulation.

        Parameter
        ---------
        spec_provider: ModelSpecProvider | None, default=None
            MuJoCo model specification provider instance to use for loading model specifications.

        substeps: int, default=4
            The number of simulation substeps between the agent update cycles.

        vision_interval: int, default=1
            The interval in which vision perception will be generated.
        """

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

    @property
    def timestep(self) -> float:
        """The length of a simulation step (as perceived by an agent)."""

        return float(self.mj_model.opt.timestep * self.n_substeps)

    @property
    @abstractmethod
    def sim_objects(self) -> Iterator[SimObject]:
        """Iterator over all simulation objects."""

    @property
    @abstractmethod
    def sim_agents(self) -> Iterator[SimAgent]:
        """Iterator over all simulation agent representation."""

    def kill_sim(self) -> None:
        """Kill the simulation (server)."""

        # TODO: Implement some sort of kill-flag that can be evaluated in external components (mainly the sim server) to trigger a shutdown.

    def ctrl_motor(
        self,
        name: str,
        q: float,
        dq: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        """Command a motor movement, which produces a torque on the actuator via a PD controller:

        ``applied_torque = kp * (q - q_current) + kd * (dq - dq_current) + tau``

        Parameter
        ---------
        name : str
            The name of the motor actuator.

        q : float
            The target position of the actuator.

        dq : float
            The target velocity of the actuator.

        kp : float
            The proportional gain of the actuator.

        kd : float
            The derivative gain of the actuator.

        tau : float
            The torque of the actuator.
        """

        actuator_tau_model = self.mj_model.actuator(name + '_tau')
        actuator_tau_data = self.mj_data.actuator(name + '_tau')
        actuator_pos_model = self.mj_model.actuator(name + '_pos')
        actuator_pos_data = self.mj_data.actuator(name + '_pos')
        actuator_vel_model = self.mj_model.actuator(name + '_vel')
        actuator_vel_data = self.mj_data.actuator(name + '_vel')

        if actuator_tau_model is not None:
            actuator_tau_data.ctrl = tau
            actuator_pos_data.ctrl = q
            actuator_vel_data.ctrl = dq
            actuator_pos_model.gainprm[0] = kp
            actuator_pos_model.biasprm[1] = -kp
            actuator_vel_model.gainprm[0] = kd
            actuator_vel_model.biasprm[2] = -kd

    def init(self) -> bool:
        """Initialize the game and create a new simulation world environment."""

        # initialize game and create game world environment
        self._mj_spec = self._create_world()
        if self._mj_spec is None:
            logger.warning('Failed to initialize game.')
            return False

        # prepare initial simulation model and data
        self._mj_model = self._mj_spec.compile()
        self._mj_data = mujoco.MjData(self._mj_model)

        # calculate forward kinematics / dynamics of newly created world
        mujoco.mj_forward(self._mj_model, self._mj_data)

        # extract visible object markers of world
        self._world_markers = [(site.name, site.name[:-10]) for site in self._mj_spec.sites if site.name.endswith('-vismarker')]

        # reset frame id
        self._frame_id = 0

        return True

    def shutdown(self) -> None:
        """Shutdown simulation."""

        self._mj_spec = None
        self._mj_model = None
        self._mj_data = None
        self._world_markers = []

    def add_agents(self, params: Sequence[PAgentParameter]) -> list[SimAgent | None]:
        """Try adding agent representations for the given list of agent params.

        Parameter
        ---------
        params: Sequence[PAgentParameter]
            The list of parameter for which to add agents.
        """

        # add agents
        sim_agents = [self._add_agent(p) for p in params]

        if sim_agents:
            # recompile spec in case new agents got added
            self._recompile()

        return sim_agents

    def _add_agent(self, params: PAgentParameter) -> SimAgent | None:
        """Try to add an agent with the given parameter.

        Parameter
        ---------
        params: PAgentParameter
            The agent parameter.
        """

        # try to load the robot model requested by the agent
        robot_spec = self.spec_provider.load_robot(params.model_name)
        if robot_spec is None:
            # failed to load the requested model --> report failure
            return None

        # request participation in game
        sim_agent = self._request_participation(params.team_name, params.player_no, robot_spec)
        if sim_agent is None:
            # invalid team side -> report failure
            return None

        # append robot to world
        frame = self._mj_spec.worldbody.add_frame()
        frame.attach_body(robot_spec.body('torso'), sim_agent.agent_id.prefix, '')

        logger.info('Player %s #%d joined the game.', params.team_name, params.player_no)

        return sim_agent

    def remove_agents(self, agents: Sequence[SimAgent]) -> None:
        """Remove the given list of agents.

        Parameter
        ---------
        agents: Sequence[SimAgent]
            The list of agent instances to remove.
        """

        if agents:
            for agent in agents:
                self._remove_agent(agent)

            self._recompile()

    def _remove_agent(self, agent: SimAgent) -> None:
        """Remove the given agent instance.

        Parameter
        ---------
        agent: SimAgent
            The simulation agent to remove.
        """

        # remove agent model from simulation
        self._mj_spec.delete(self._mj_spec.body(agent.root_body_name))

        # delete various components manually, as they are not automatically removed again when the root body is detached
        # Note:
        # Not sure if this is intentional behavior or a bug in mujoco.
        # It's also not clear what components need to be deleted separately.
        # The code below so far prevents any follow-up exceptions when re-attaching the same model again.
        # But at the moment, there is no guarantee that there will be no components left in the spec that may cause some trouble at some point.
        def del_els(el_list: list[Any]) -> None:
            for el in el_list:
                self._mj_spec.delete(el)

        # del_els(agent.spec.cameras)
        # del_els(agent.spec.geoms)
        # del_els(agent.spec.lights)
        del_els(agent.spec.materials)
        del_els(agent.spec.meshes)
        del_els(agent.spec.sites)
        del_els(agent.spec.texts)
        del_els(agent.spec.textures)

        # remove agent from game
        self._handle_withdrawal(agent.agent_id)

        logger.info('Player %s left the game.', agent)

    def _recompile(self) -> None:
        """Recompile spec and bind sim objects."""

        # recompile spec
        self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)

        # calculate forward kinematics / dynamics
        mujoco.mj_forward(self._mj_model, self._mj_data)

        # rebind objects
        for obj in self.sim_objects:
            obj.bind(self._mj_model, self._mj_data)

    def step(self, agent_actions: Sequence[SimAction], monitor_commands: Sequence[MonitorCommand]) -> None:
        """Perform a simulation step.

        Parameter
        ---------
        agent_actions: Sequence[SimAction]
            The list of simulation agent actions.

        monitor_commands: Sequence[MonitorCommand]
            The list of monitor commands.
        """

        # pre-step hook
        self._pre_step(agent_actions)

        # progress simulation
        mujoco.mj_step(self._mj_model, self._mj_data, self.n_substeps)

        # post-step hook
        self._post_step(monitor_commands)

        # increment frame id
        self._frame_id += 1

    def _pre_step(self, agent_actions: Sequence[SimAction]) -> None:
        """Method executed right before a simulation step.

        Parameter
        ---------
        agent_actions: Sequence[SimAction]
            The list of simulation actions.
        """

        # notify simulation objects
        for obj in self.sim_objects:
            obj.pre_step(self._mj_model, self._mj_data)

        # apply agent actions
        for action in agent_actions:
            action.perform(self)

    def _post_step(self, monitor_commands: Sequence[MonitorCommand]) -> None:
        """Method executed right after a simulation step.

        Parameter
        ---------
        monitor_commands: Sequence[MonitorCommand]
            The list of monitor commands.
        """

        # notify simulation objects
        for obj in self.sim_objects:
            obj.post_step(self._mj_model, self._mj_data)

        # apply monitor commands
        for command in monitor_commands:
            command.perform(self)

    def generate_perceptions(self, *, gen_vision: bool | None = None) -> None:
        """Generate perceptions for active agents.

        Parameter
        ---------
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

        # fetch all active agents
        agents = [*self.sim_agents]

        # generate general perceptions equal for all agents
        sim_time_perception = TimePerception('now', trunc2(self._mj_data.time))
        game_state_perception = self._generate_game_state_perception()

        if gen_vision:
            # collect visible markers
            n_world_markers = len(self._world_markers)
            obj_markers = list(self._world_markers)
            for player in agents:
                obj_markers.extend(player.markers)

            # extract visible object positions
            n_markers = len(obj_markers)
            obj_pos = np.zeros((3, n_markers), dtype=np.float64)
            for idx, site in enumerate(obj_markers):
                obj_pos[:, idx] = self._mj_data.site(site[0]).xpos.astype(np.float64)

        # generate agent specific perceptions
        for agent in agents:
            joint_names: list[str] = []
            joint_axs: list[float] = []
            joint_vxs: list[float] = []
            agent_perceptions: list[Perception] = [sim_time_perception, game_state_perception]

            prefix_length = len(agent.agent_id.prefix)

            for sensor_spec in agent.spec.sensors:
                sensor = self._mj_data.sensor(sensor_spec.name)
                sensor_name = sensor_spec.name[prefix_length:]

                if sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    joint_names.append(sensor_name)
                    joint_axs.append(sensor.data[0])

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTVEL:
                    joint_vxs.append(sensor.data[0])

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_GYRO:
                    rvx, rvy, rvz = trunc2_vec(np.degrees(sensor.data[0:3]))
                    agent_perceptions.append(GyroPerception(sensor_name, rvx, rvy, rvz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                    ax, ay, az = trunc2_vec(sensor.data[0:3])
                    agent_perceptions.append(AccelerometerPerception(sensor_name, ax, ay, az))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_TOUCH:
                    active = int(sensor.data[0])
                    agent_perceptions.append(TouchPerception(sensor_name, active))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEQUAT:
                    qw, qx, qy, qz = trunc3_vec(sensor.data[0:4])
                    agent_perceptions.append(OrientationPerception(sensor_name, qw, qx, qy, qz))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_FRAMEPOS:
                    px, py, pz = trunc3_vec(sensor.data[0:3])
                    agent_perceptions.append(PositionPerception(sensor_name, px, py, pz))

                # TODO: Add perceptions for force and hear

                else:
                    # sensor not supported...
                    pass

            # joint state perception
            if joint_names:
                agent_perceptions.append(JointStatePerception(joint_names, trunc2_vec(np.degrees(joint_axs)), trunc2_vec(np.degrees(joint_vxs))))

            # ideal camera sensor-pipeline
            if gen_vision:
                # fetch camera sensor site
                camera_site = self._mj_data.site(agent.agent_id.prefix + 'camera')

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

                    # extract player object detections
                    idx = n_world_markers
                    for player in agents:
                        n_player_markers = len(player.markers)
                        player_detections = [ObjectDetection(obj_markers[i][1], azimuths[i], elevations[i], distances[i]) for i in range(idx, idx + n_player_markers) if obj_visibility[i]]
                        if player_detections:
                            obj_detections.append(AgentDetection('P', player.team_name, player.agent_id.player_no, player_detections))

                        idx += n_player_markers

                    agent_perceptions.append(VisionPerception('See', obj_detections))

            # forward generated perceptions to agent instance
            agent.set_perceptions(agent_perceptions)

    def generate_state_information(self) -> list[SimStateInformation]:
        """Generate simulation state information for updating monitor instances."""

        return [SceneGraph(self._mj_model, self._mj_data)]

    @abstractmethod
    def _create_world(self) -> Any | None:
        """Create a new simulation world environment.

        Returns
        -------
        MjSpec
            The game specific simulation world / environment specification.
        """

    @abstractmethod
    def _request_participation(self, team_name: str, player_no: int, model_spec: Any) -> SimAgent | None:
        """Validate and add a new agent requesting to join the game.

        Parameter
        ---------
        team_name: str
            The team name of the agent requesting participation.

        player_no: int
            The player number of the agent requesting participation.

        model_spec: MjSpec
            The robot model specification of the new agent.

        Returns
        -------
        SimAgent | None
            The simulation agent representation identifying the new agent if participation has been accepted, None if participation of the new agent has been rejected.
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
