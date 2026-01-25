.. _agent-protocol:

Agent Protocol
==============

This chapter describes the communication protocol used to exchange information between the simulation server and agents.
The communication protocol is separated into Perceptors - which represent information an agent can receive - and Effectors - which represent information an agent can send.

When using the :ref:`Simulation Server <sim-server>`, information exchange is realized via its TCP/IP based :ref:`network protocol <sim-server_network-protocol>` using specific :ref:`message formats <sim-server_message-formats>`.
In case of the :ref:`Managed Simulation <managed-sim>`, information is typically exchanged using simple Python data objects.


.. _agent-protocol_perceptors:

Perceptors
----------

Perceptors in general represent information an agent can receive.
Perceptors are typically related to sensors of the :ref:`robot model <robot-models>` of an agent, but can represent arbitrary information.
They allow the agent to perceive state information about itself and the simulation environment.
The available simulation perceptors are listed below.


.. _agent-protocol_time-perceptor:

Time Perceptor
^^^^^^^^^^^^^^

General time perceptor, usually used to perceive the simulation / world time.

+-------------+-------+---------+-------------------------------------------------------+
| Perceptor   | Type  | Unit    | Description                                           |
| Information |       |         |                                                       |
+=============+=======+=========+=======================================================+
| **name**    | str   |         | A name identifying the time reference.                |
+-------------+-------+---------+-------------------------------------------------------+
| **time**    | float | seconds | The time.                                             |
+-------------+-------+---------+-------------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.TimePerception`                                   |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the time is truncated to three digits (milliseconds).


Message Format
""""""""""""""

S-Expression:
    .. code::

        (time (<name> <time>))


    Example message:

    .. code::

        (time (now 1.2))


.. _agent-protocol_position-perceptor:

Position Perceptor
^^^^^^^^^^^^^^^^^^

Virtual perceptor, representing the position of an object in the world.

+-------------+-------+-------+---------------------------------------------------------+
| Perceptor   | Type  | Unit  | Description                                             |
| Information |       |       |                                                         |
+=============+=======+=======+=========================================================+
| **name**    | str   |       | The name of the object / reference frame.               |
+-------------+-------+-------+---------------------------------------------------------+
| **x**       | float | meter | The x-coordinate of the position.                       |
+-------------+-------+-------+---------------------------------------------------------+
| **y**       | float | meter | The y-coordinate of the position.                       |
+-------------+-------+-------+---------------------------------------------------------+
| **z**       | float | meter | The z-coordinate of the position.                       |
+-------------+-------+-------+---------------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.PositionPerception`                               |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the position coordinates are truncated to three digits (millimeter).


Message Format
""""""""""""""

S-Expression:
    .. code::

        (pos (n <name>) (pos <x> <y> <z>))

    Example message:

    .. code::

        (pos (n torso_pos) (pos -0.122 24.575 0.762))


.. _agent-protocol_orientation-perceptor:

Orientation Perceptor
^^^^^^^^^^^^^^^^^^^^^

Virtual perceptor, representing the orientation of an object in the world.

+-------------+-------+------+------------------------------------------------------------+
| Perceptor   | Type  | Unit | Description                                                |
| Information |       |      |                                                            |
+=============+=======+======+============================================================+
| **name**    | str   |      | The name of the object / reference frame.                  |
+-------------+-------+------+------------------------------------------------------------+
| **qw**      | float |      | The w-coordinate of the orientation quaternion.            |
+-------------+-------+------+------------------------------------------------------------+
| **qx**      | float |      | The x-coordinate of the orientation quaternion.            |
+-------------+-------+------+------------------------------------------------------------+
| **qy**      | float |      | The y-coordinate of the orientation quaternion.            |
+-------------+-------+------+------------------------------------------------------------+
| **qz**      | float |      | The z-coordinate of the orientation quaternion.            |
+-------------+-------+------+------------------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.OrientationPerception`                              |
+-------------+-------------+-----------+-------------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the quaternion coordinates are truncated to three digits.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (quat (n <name>) (q <qw> <qx> <qy> <qz>))

    Example message:

    .. code::

        (quat (n torso_quat) (q 1.0 0.0 0.0 0.0))


.. _agent-protocol_gyro-rate-perceptor:

Gyro-Rate Perceptor
^^^^^^^^^^^^^^^^^^^

Perceptor representing a gyro-rate sensor.

+-------------+-------+-----------+-----------------------------------------------------+
| Perceptor   | Type  | Unit      | Description                                         |
| Information |       |           |                                                     |
+=============+=======+===========+=====================================================+
| **name**    | str   |           | The unique name of the sensor.                      |
+-------------+-------+-----------+-----------------------------------------------------+
| **rx**      | float | deg / sec | The rotational velocity around the x-axis.          |
+-------------+-------+-----------+-----------------------------------------------------+
| **ry**      | float | deg / sec | The rotational velocity around the y-axis.          |
+-------------+-------+-----------+-----------------------------------------------------+
| **rz**      | float | deg / sec | The rotational velocity around the z-axis.          |
+-------------+-------+-----------+-----------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.GyroPerception`                                   |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the angular velocity values are truncated to two digits.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (GYR (n <name>) (rt <rx> <ry> <rz>))

    Example message:

    .. code::

        (GYR (n torso_gyro) (rt -6.97 -3.31 25.16))


.. _agent-protocol_accelerometer-perceptor:

Accelerometer Perceptor
^^^^^^^^^^^^^^^^^^^^^^^

Perceptor representing an accelerometer sensor.

+-------------+-------+-----------+-----------------------------------------------------+
| Perceptor   | Type  | Unit      | Description                                         |
| Information |       |           |                                                     |
+=============+=======+===========+=====================================================+
| **name**    | str   |           | The unique name of the sensor.                      |
+-------------+-------+-----------+-----------------------------------------------------+
| **ax**      | float | m / sec^2 | The acceleration along the x-axis.                  |
+-------------+-------+-----------+-----------------------------------------------------+
| **ay**      | float | m / sec^2 | The acceleration along the y-axis.                  |
+-------------+-------+-----------+-----------------------------------------------------+
| **az**      | float | m / sec^2 | The acceleration along the z-axis.                  |
+-------------+-------+-----------+-----------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.AccelerometerPerception`                          |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the linear acceleration values are truncated to two digits.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (ACC (n <name>) (a <ax> <ay> <az>))

    Example message:

    .. code::

        (ACC (n torso_acc) (a 0.13 0.41 -9.75))


.. _agent-protocol_joint-state-perceptor:

Joint State Perceptor
^^^^^^^^^^^^^^^^^^^^^

Perceptor representing a set of joint sensors, sensing joint position and velocity.

+-------------+-------------+-----------+-----------------------------------------------+
| Perceptor   | Type        | Unit      | Description                                   |
| Information |             |           |                                               |
+=============+=============+===========+===============================================+
| **name**    | str         |           | The name of the joint set.                    |
+-------------+-------------+-----------+-----------------------------------------------+
| **names**   | [str,...]   |           | The list of joint names.                      |
+-------------+-------------+-----------+-----------------------------------------------+
| **axs**     | [float,...] | deg       | The list of joint positions.                  |
+-------------+-------------+-----------+-----------------------------------------------+
| **vxs**     | [float,...] | deg / sec | The list of joint velocities.                 |
+-------------+-------------+-----------+-----------------------------------------------+
| :py:class:`rcsssmj.agent.perception.JointStatePerception`                             |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None. However, the axis position and velocity values are truncated to two digits.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (HJ (n <name>) (ax <ax>) (vx <vx>))*

    Example message:

    .. code::

        (HJ (n hj1) (ax 1.43) (vx 0.03)) (HJ (n hj2) (ax 16.92) (vx 1.44)) ...


.. _agent-protocol_touch-perceptor:

Touch Perceptor
^^^^^^^^^^^^^^^

Perceptor representing a touch / bumper sensor.

+-------------+------+------+-----------------------------------------------------------+
| Perceptor   | Type | Unit | Description                                               |
| Information |      |      |                                                           |
+=============+======+======+===========================================================+
| **name**    | str  |      | The unique name of the sensor.                            |
+-------------+------+------+-----------------------------------------------------------+
| **active**  | int  | bool | - 0 for no contact;                                       |
|             |      |      | - everything else for active contact.                     |
+-------------+------+------+-----------------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.TouchPerception`                                  |
+-------------+-------------+-----------+-----------------------------------------------+

Frequency
    Every cycle.

Noise Model
    None.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (TCH n <name> val <active>)

    Example message:

    .. code::

        (TCH n bumper val 1)


.. _agent-protocol_game-state-perceptor:

Game State Perceptor
^^^^^^^^^^^^^^^^^^^^

The (soccer) game state perceptor.

+-----------------+-------+---------+---------------------------------------------------+
| Perceptor       | Type  | Unit    | Description                                       |
| Information     |       |         |                                                   |
+=================+=======+=========+===================================================+
| **name**        | str   |         | 'GS'                                              |
+-----------------+-------+---------+---------------------------------------------------+
| **play time**   | float | seconds | The current play time.                            |
+-----------------+-------+---------+---------------------------------------------------+
| **play mode**   | str   |         | The currently active play mode.                   |
+-----------------+-------+---------+---------------------------------------------------+
| **team left**   | str   |         | The name of the left team.                        |
+-----------------+-------+---------+---------------------------------------------------+
| **team right**  | str   |         | The name of the right team.                       |
+-----------------+-------+---------+---------------------------------------------------+
| **score left**  | int   | goals   | The score (number of goals) of the left team.     |
+-----------------+-------+---------+---------------------------------------------------+
| **score right** | int   | goals   | The score (number of goals) of the right team.    |
+-----------------+-------+---------+---------------------------------------------------+
| :py:class:`rcsssmj.games.soccer.agent.perception.GameStatePerception`                 |
+-------------+-------------+-----------+-----------------------------------------------+

Available play modes are specified in :py:class:`rcsssmj.games.soccer.play_mode.PlayMode`.

Frequency
    Every cycle.

Noise Model
    None.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (GS (t <play time>) (pm <play mode>) (tl <team left>) (tr <team right>) (sl <score left>) (sr <score right>))

    Example message:

    .. code::

        (GS (t 231.52) (pm PlayOn) (tl teamBlue) (tr teamRed) (sl 2) (sr 1))


.. _agent-protocol_vision-perceptor:

Vision Perceptor
^^^^^^^^^^^^^^^^

The (ideal) vision perceptor.

+----------------+-----------------------+------+---------------------------------------+
| Perceptor      | Type                  | Unit | Description                           |
| Information    |                       |      |                                       |
+================+=======================+======+=======================================+
| **name**       | str                   |      | The unique name of the camera sensor. |
+----------------+-----------------------+------+---------------------------------------+
| **detections** | [ObjectDetection,...] |      | The list of object detections.        |
+----------------+-----------------------+------+---------------------------------------+
| :py:class:`rcsssmj.agent.perception.VisionPerception`                                 |
+----------------+-----------------------+------+---------------------------------------+

Frequency
    Every second cycle.


.. _agent-protocol_point-detection:

Point Object Detection
""""""""""""""""""""""

Point object detections are provided in polar coordinates (specifying the distance, azimuth and elevation values) where the origin is facing the x-axis.

+---------------+-------+-------+-------------------------------------------------------+
| Point Object  | Type  | Unit  | Description                                           |
| Detection     |       |       |                                                       |
+===============+=======+=======+=======================================================+
| **name**      | str   |       | The unique name of the detected object.               |
+---------------+-------+-------+-------------------------------------------------------+
| **distance**  | float | meter | The distance to the detected point.                   |
+---------------+-------+-------+-------------------------------------------------------+
| **azimuth**   | float | deg   | The azimuth / horizontal angle to the detected point. |
+---------------+-------+-------+-------------------------------------------------------+
| **elevation** | float | deg   | The elevation / vertical angle to the detected point. |
+---------------+-------+-------+-------------------------------------------------------+
| :py:class:`rcsssmj.agent.perception.ObjectDetection`                                  |
+---------------+-------+-------+-------------------------------------------------------+

Noise Model
    None. However, direction angles and distance values are truncated to two digits.


.. _agent-protocol_agent-detection:

Agent Detection
"""""""""""""""

Each robot models specifies a set of visible markers that are represented as individual point object detections within an agent detection.

+---------------------+----------------------------+------+--------------------------------------------+
| Agent Detection     | Type                       | Unit | Description                                |
+=====================+============================+======+============================================+
| **name**            | str                        |      | 'P'                                        |
+---------------------+----------------------------+------+--------------------------------------------+
| **team name**       | str                        |      | The name of the team the agent belongs to. |
+---------------------+----------------------------+------+--------------------------------------------+
| **player no**       | int                        |      | The player number.                         |
+---------------------+----------------------------+------+--------------------------------------------+
| **body detections** | [PointObjectDetection,...] |      | The list of detected body part markers.    |
+---------------------+----------------------------+------+--------------------------------------------+
| :py:class:`rcsssmj.agent.perception.AgentDetection`                                                  |
+---------------------+----------------------------+------+--------------------------------------------+


Message Format
""""""""""""""

S-Expression:
    .. code::

        (See <detections>)

    Point object detection:
        .. code::

            (<name> (pol <azimuth> <elevation> <distance>))

    Agent detection:
        .. code::

            (P (team <team name>) (id <player no>) <body detections>)

    Example message:

    .. code::

        (See (G2R (pol 17.55 -3.33 4.31))
             (G1R (pol 17.52 3.27 4.07))
             (F1R (pol 18.52 18.94 1.54))
             (F2R (pol 18.52 -18.91 1.52))
             (B (pol 8.51 -0.21 -0.17))
             (P (team teamRed) (id 1)
                (head (pol 16.98 -0.21 3.19))
                (rlowerarm (pol 16.83 -0.06 2.80))
                (llowerarm (pol 16.86 -0.36 3.10))
                (rfoot (pol 17.00 0.29 1.68))
                (lfoot (pol 16.95 -0.51 1.32)))
             (P (team teamBlue) (id 3)
                (rlowerarm (pol 0.18 -33.55 -20.16))
                (llowerarm (pol 0.18 34.29 -19.80))))


.. _agent-protocol_effectors:

Effectors
---------

Effectors in general represent actions an agent can perform (send).
They allow the agent to manipulate its state within the simulation environment.
The available simulation effectors are listed below.


.. _agent-protocol_init-request:

Init Request
^^^^^^^^^^^^

The agent initialization request.
This request has to be sent as the very first message of a newly connected client to initialize an agent within the simulation environment.

Perceptor information is only sent after a client has successfully initialized itself.
Similarly, effector actions are only processed after successful initialization.

+----------------+------+------+------------------------------------------------------------+
| Effector       | Type | Unit | Description                                                |
| Information    |      |      |                                                            |
+================+======+======+============================================================+
| **model name** | str  |      | The unique name of the robot model to load for this agent. |
+----------------+------+------+------------------------------------------------------------+
| **team name**  | str  |      | The requested team name of the agent.                      |
+----------------+------+------+------------------------------------------------------------+
| **player no**  | int  |      | The requested player number.                               |
+----------------+------+------+------------------------------------------------------------+
| :py:class:`rcsssmj.agent.action.InitRequest`                                              |
+----------------+------+------+------------------------------------------------------------+


Message Format
""""""""""""""

S-Expression:
    .. code::

        (init <model name> <team name> <player no>)

    Example message:

    .. code::

        (init T1 teamBlue 2)


.. _agent-protocol_beam-effector:

Beam Effector
^^^^^^^^^^^^^

Effector for placing an agent at a certain 2D pose within the simulation world.

This effector is intended to accelerate game setup and typically only respected in certain play modes of the game.

When performing a beam action, the agent is not guaranteed to be placed at the requested location.
Depending on the game situation, there might be restrictions to certain areas in the world where an agent is allowed to beam.
Apart from that, the requested beam location might already be occupied by another agent / object.
In such a the case, the requested beam location might be arbitrary altered by the simulation server.

In case of the soccer simulation, the beam location is always interpreted from the playing side.
This means that both teams specify beam locations relative to the left field side and agents of the right team are automatically transformed to the right side.

+-----------------+-------+-------+-----------------------------------------------------+
| Effector        | Type  | Unit  | Description                                         |
| Information     |       |       |                                                     |
+=================+=======+=======+=====================================================+
| **name**        | str   |       | The unique name of the beam actuator.               |
+-----------------+-------+-------+-----------------------------------------------------+
| **x**           | float | meter | The x-coordinate of the target position.            |
+-----------------+-------+-------+-----------------------------------------------------+
| **y**           | float | meter | The y-coordinate of the target position.            |
+-----------------+-------+-------+-----------------------------------------------------+
| **theta**       | float | deg   | The horizontal orientation / angle.                 |
+-----------------+-------+-------+-----------------------------------------------------+
| :py:class:`rcsssmj.games.soccer.agent.action.BeamAction`                              |
+-----------------+-------+-------+-----------------------------------------------------+

Noise Model
    None so far.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (beam <x> <y> <theta>)

    Example message:

    .. code::

        (beam -29.5 16 -35.0)


.. _agent-protocol_motor-effector:

Motor / Joint Effector
^^^^^^^^^^^^^^^^^^^^^^

Effector for controlling a motor of the robot model associated with the agent in the simulation.

+-------------+-------+-----------+-----------------------------------------------------+
| Effector    | Type  | Unit      | Description                                         |
| Information |       |           |                                                     |
+=============+=======+===========+=====================================================+
| **name**    | str   |           | The unique name of the joint actuator.              |
+-------------+-------+-----------+-----------------------------------------------------+
| **q**       | float | deg       | The joint target position.                          |
+-------------+-------+-----------+-----------------------------------------------------+
| **dq**      | float | deg / sec | The joint target velocity.                          |
+-------------+-------+-----------+-----------------------------------------------------+
| **kp**      | float |           | The position gain parameter.                        |
+-------------+-------+-----------+-----------------------------------------------------+
| **kd**      | float |           | The velocity gain parameter.                        |
+-------------+-------+-----------+-----------------------------------------------------+
| **tau**     | float | Nm        | The extra torque.                                   |
+-------------+-------+-----------+-----------------------------------------------------+
| :py:class:`rcsssmj.agent.action.MotorAction`                                          |
+-------------+-------+-----------+-----------------------------------------------------+

Noise Model
    None.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (<name> <q> <dq> <kp> <kd> <tau>)

    Example message:

    .. code::

        (he1 12.42 0 0.9 0 0)


.. _agent-protocol_say-effector:

Say Effector
^^^^^^^^^^^^

Effector for exchanging text messages within the simulation in an audio-like fashion.

Agents have a certain say capacity, which they can use to broadcast text messages in the simulation.
These text messages are then received by other agents within the audible range.

.. note::

    Not implemented yet.


Message Format
""""""""""""""

S-Expression:
    .. code::

        (say <message>)

    Example message:

    .. code::

        (say HelloWorld)
