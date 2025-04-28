import socket
from math import degrees
from queue import Empty
from threading import Thread
import time
from typing import TYPE_CHECKING, Any, Final, cast

import mujoco

from rcsssmj.agent import AgentID
from rcsssmj.client.perception import AccelerometerPerception, GyroPerception, JointPerception, Perception, TimePerception, TouchPerception
from rcsssmj.client.sim_client import SimClient, SimClientState
from rcsssmj.communication.tcp_lpm_connection import TCPLPMConnection
from rcsssmj.game.referee import SoccerReferee
from rcsssmj.monitor.monitor_client import MonitorClient
from rcsssmj.monitor.mujoco_monitor import MujocoMonitor
from rcsssmj.resources.spec_provider import ModelSpecProvider

if TYPE_CHECKING:
    from rcsssmj.client.action import SimAction
    from rcsssmj.monitor.sim_monitor import SimMonitor


class Server:
    def __init__(
            self,
            host: str,
            client_port: int = 60000,
            monitor_port: int = 60001,
            *,
            render: bool = True
        ) -> None:
        """
        Construct a new simulation sever.
        """

        self.host: Final[str] = host
        self.client_port: Final[int] = client_port
        self.monitor_port: Final[int] = monitor_port
        self.render: Final[bool] = render
        self.sync_mode: Final[bool] = False
        self.real_time: Final[bool] = True

        self._spec_provider = ModelSpecProvider()
        self._referee: SoccerReferee = SoccerReferee()
        self._clients: list[SimClient] = []
        self._monitors: list[SimMonitor] = []

        self._client_sock: socket.socket | None = None
        self._monitor_sock: socket.socket | None = None

        self._shutdown: bool = True

    def run(self) -> None:
        """
        Run simulation server.
        """

        if self._client_sock is not None or self._monitor_sock is not None:
            # a simulation is already running...
            raise RuntimeError

        # 1. SETUP: Setup sockets and start server threads
        print("[STARTING] Server is starting...")
        self._shutdown = False

        # setup client socket
        self._client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._client_sock.bind((self.host, self.client_port))
        self._client_sock.listen(5)

        # setup monitor socket
        try:
            self._monitor_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._monitor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._monitor_sock.bind((self.host, self.monitor_port))
            self._monitor_sock.listen(5)
        except ConnectionError:
            self._client_sock.shutdown(socket.SHUT_RDWR)
            self._client_sock.close()
            raise

        # create simulator threads
        sim_thread = Thread(target=self._run_simulation, name='sim_loop')
        client_listener_thread = Thread(target=self._listen_for_clients, name='client_connections_listener')
        monitor_listener_thread = Thread(target=self._listen_for_monitors, name='monitor_connections_listener')

        # start simulator threads
        client_listener_thread.start()
        monitor_listener_thread.start()
        sim_thread.start()

        print("[STARTING] DONE!")


        # 2. RUN: Wait until simulation thread finished
        sim_thread.join()  # run simulation loop in separate thread to isolate exceptions and allow the main thread to clean up


        # 3. CLEANUP: Shutdown everything and wait for socket threads to finish
        print("[SHUTDOWN] Server is shutting down...")
        self._shutdown = True

        # shutdown client and monitor sockets
        try:
            self._client_sock.shutdown(socket.SHUT_RDWR)
        except:
            pass

        try:
            self._monitor_sock.shutdown(socket.SHUT_RDWR)
        except:
            pass

        # wait for client and monitor connection listener threads to finish
        client_listener_thread.join()
        monitor_listener_thread.join()

        # shutdown active clients
        for client in self._clients:
            client.shutdown(no_wait=False)
        self._clients.clear()
        print("[SHUTDOWN] - Clients.")

        # shutdown active monitors
        for monitor in self._monitors:
            monitor.shutdown(no_wait=False)
        self._monitors.clear()
        print("[SHUTDOWN] - Monitors.")

        # cleanup socket refs
        self._client_sock = None
        self._monitor_sock = None

        print("[SHUTDOWN] DONE!")

    def shutdown(self) -> None:
        """
        Request server shutdown.
        """

        self._shutdown = True

        print("[INFO] Shutdown requested.")

    def _listen_for_clients(self) -> None:
        """
        Wait for incoming client connections.

        Method executed by client listener thread.
        """

        if self._client_sock is None:
            return

        print("[STARTING] - Client connection listener...")
        while not self._shutdown:
            try:
                sock, addr = self._client_sock.accept()
            except:
                self._shutdown = True
                break

            print(f"[CLIENT] {addr} connected.")

            conn = TCPLPMConnection(sock, addr)
            client = SimClient(conn)

            # TODO: synchronize threads when accessing client list
            self._clients.append(client)

        print("[SHUTDOWN] - Client listener thread.")
        self._client_sock.close()

    def _listen_for_monitors(self) -> None:
        """
        Wait for incoming monitor connections.

        Method executed by monitor listener thread.
        """

        if self._monitor_sock is None:
            return

        print("[STARTING] - Monitor connection listener...")
        while not self._shutdown:
            try:
                sock, addr = self._monitor_sock.accept()
            except:
                self._shutdown = True
                break

            print(f"[MONITOR] {addr} connected.")

            conn = TCPLPMConnection(sock, addr)
            monitor = MonitorClient(conn)

            # TODO: synchronize threads when accessing monitor list
            self._monitors.append(monitor)

        print("[SHUTDOWN] - Monitor listener thread.")
        self._monitor_sock.close()

    def _run_simulation(self) -> None:
        """
        Simulation main loop.

        Method executed by main thread.
        """

        print("[STARTING] - Simulation loop...")

        # load soccer pitch spec
        spec = self._spec_provider.load_environment('soccer')
        if spec is None:
            return

        # prepare initial simulation model and data
        mj_model = spec.compile()
        mj_data = mujoco.MjData(mj_model)
        n_substeps = 5
        sim_timestep: float = mj_model.opt.timestep * n_substeps

        # create internal monitor
        if self.render:
            self._monitors.append(MujocoMonitor(mj_model, 2))

        needs_recompile: bool = False
        cycle_start: float = time.time() - sim_timestep

        # simulation loop
        while not self._shutdown:
            # filter clients / monitors by state, as their state may change during this simulation step
            # this also simplifies client / monitor removal from the central client / monitor lists
            ready_clients: list[SimClient] = []
            active_clients: list[SimClient] = []
            disconnected_clients: list[SimClient] = []
            clients_to_remove: list[SimClient] = []
            activated_clients: list[SimClient] = []

            active_monitors: list[SimMonitor] = []
            monitors_to_remove: list[SimMonitor] = []

            client_actions: list[SimAction] = []

            # TODO: synchronize threads when accessing client list
            for client in self._clients:
                if client.get_state() == SimClientState.READY:
                    ready_clients.append(client)
                if client.get_state() == SimClientState.ACTIVE:
                    active_clients.append(client)
                if client.get_state() == SimClientState.DISCONNECTED:
                    disconnected_clients.append(client)
                    clients_to_remove.append(client)
                else:
                    # CONNECTED state not relevant here
                    pass

            # TODO: synchronize threads when accessing monitor list
            for monitor in self._monitors:
                if monitor.is_active():
                    active_monitors.append(monitor)
                else:
                    monitors_to_remove.append(monitor)

            # handle disconnected clients
            for client in disconnected_clients:
                # check if client has been activated before
                agent_id = client.get_id()
                if agent_id is not None:
                    print(f'[TEAM {client.get_team_name()}] Player {client.get_player_no()} left the game.')

                    # remove client model from simulation
                    spec.detach_body(spec.body(agent_id.prefix + 'torso'))
                    needs_recompile = True

                    # remove agent from game
                    self._referee.handle_withdrawal(agent_id)

            # handle ready clients
            for client in ready_clients:
                # try to load the robot model requested by the client
                robot_spec = self._spec_provider.load_robot(client.get_model_name())
                if robot_spec is None:
                    # failed to load the requested model
                    client.shutdown()
                    clients_to_remove.append(client)
                    continue

                # request participation in game
                agent_id = self._referee.request_participation(client, robot_spec)
                if agent_id is None:
                    # invalid team side -> shutdown client
                    client.shutdown()
                    clients_to_remove.append(client)
                    continue

                print(f'[TEAM {client.get_team_name()}] Player {client.get_player_no()} joined the game.')

                # append robot to world
                frame = spec.worldbody.add_frame()
                frame.attach_body(robot_spec.body('torso'), agent_id.prefix, '')
                needs_recompile = True

                client.activate(agent_id, robot_spec)
                activated_clients.append(client)
                active_clients.append(client)

            if needs_recompile:
                mj_model, mj_data = spec.recompile(mj_model, mj_data)
                needs_recompile = False

            # initialize newly activated players
            for client in activated_clients:
                self._referee.spawn_agent(cast(AgentID, client.get_id()), mj_data)

            # generate perceptions
            self._generate_perceptions(active_clients, mj_data)

            # sleep to match simulation interval
            if self.real_time:
                time.sleep(max(0, sim_timestep - (time.time() - cycle_start) - 0.0001))
                cycle_start: float = time.time()

            # collect client actions and send perceptions
            for client in active_clients:
                # collect all pending actions
                action_queue = client.get_action_queue()
                try:
                    if self.sync_mode:
                        # wait for exactly one client action
                        client_actions += action_queue.get(timeout=10)
                    else:
                        # fetch all currently available actions
                        while action_queue.qsize() > 0:
                            client_actions += action_queue.get_nowait()
                except Empty:
                    if self.sync_mode:
                        # client took too long to answer -> kill it
                        print(f'[TEAM {client.get_team_name()}] Player {client.get_player_no()} did not respond for more than 10 seconds. Forcing player shutdown.')
                        client.shutdown()
                        continue

                # send perceptions
                client.send_perceptions()

            # apply client actions
            for action in client_actions:
                action.perform(self._referee, mj_data)

            # progress simulation
            mujoco.mj_step(mj_model, mj_data, n_substeps)

            # apply monitor commands
            for monitor in active_monitors:
                command_queue = monitor.get_command_queue()
                try:
                    while command_queue.qsize() > 0:
                        command = command_queue.get_nowait()
                        command.perform(self._referee, mj_data)
                except Empty:
                    pass

            # call referee to judge the current simulation state and progress the game state
            self._referee.referee(mj_data)

            # update connected monitors
            for monitor in active_monitors:
                monitor.update(mj_model, mj_data)

            # TODO: log monitor message to simulator log
            # TODO: log client perceptions and actions to client logs

            # remove disconnected clients
            # TODO: synchronize threads when accessing client list
            for client in clients_to_remove:
                self._clients.remove(client)

            # remove inactive monitors
            # TODO: synchronize threads when accessing monitor list
            for monitor in monitors_to_remove:
                self._monitors.remove(monitor)

        print("[INFO] Simulation thread finished.")

    def _generate_perceptions(self, active_clients: list[SimClient], mj_data: Any) -> None:
        """
        Generate perceptions for active clients.
        """

        def t2(val: float) -> float:
            """Limit the given value to two digits."""
            return int(val * 100) / 100.0

        # generate general perceptions equal for all clients
        sim_time_perception = TimePerception('now', t2(mj_data.time))
        game_state_perception = self._referee.generate_perception()

        # generate client specific perceptions
        for client in active_clients:
            perceptions: list[Perception] = [sim_time_perception, game_state_perception]

            model_spec = cast(Any, client.get_model_spec())
            agent_id = cast(AgentID, client.get_id())
            prefix_length = len(agent_id.prefix)

            for sensor_spec in model_spec.sensors:
                sensor = mj_data.sensor(sensor_spec.name)
                sensor_name = sensor_spec.name[prefix_length:]

                if sensor_spec.type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    perceptions.append(JointPerception(sensor_name, t2(degrees(sensor.data[0]))))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_GYRO:
                    rvx, rvy, rvz = sensor.data
                    perceptions.append(GyroPerception(sensor_name, t2(degrees(rvx)), t2(degrees(rvy)), t2(degrees(rvz))))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                    ax, ay, az = sensor.data
                    perceptions.append(AccelerometerPerception(sensor_name, t2(ax), t2(ay), t2(az)))

                elif sensor_spec.type == mujoco.mjtSensor.mjSENS_TOUCH:
                    active = int(sensor.data[0])
                    perceptions.append(TouchPerception(sensor_name, active))

                # TODO: Add perceptions for force, vision and hear

                else:
                    # sensor not supported...
                    pass

            client.set_perceptions(perceptions)
