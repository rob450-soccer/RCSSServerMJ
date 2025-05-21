.. _robot-model_ant:

Ant Robot
=========

The `ant <https://gymnasium.farama.org/environments/mujoco/ant/>`_ robot is a simple model for testing from the `Gymnasium <https://github.com/Farama-Foundation/Gymnasium>`_ project.

+--------------------+--------------------------------------------------------------+
| Model Properties                                                                  |
+====================+==============================================================+
| **Name / ID**      | 'ant'                                                        |
+--------------------+--------------------------------------------------------------+
| **Type / Anatomy** | 4-legged robot                                               |
+--------------------+--------------------------------------------------------------+
| **DoF**            | 8 |ndash| 2 per leg                                          |
+--------------------+--------------------------------------------------------------+


.. _robot-model_ant-joints:

Joints
------

The Ant robot is equipped with the following joints:

+---------+-----------+--------------+
| Name    | Axis      | Range (deg)  |
+=========+===========+==============+
| hip_1   | (0, 0, 1) | -30 ...  30  |
+---------+-----------+--------------+
| ankle_1 | (1, 1, 0) |  30 ...  70  |
+---------+-----------+--------------+
| hip_2   | (0, 0, 1) | -30 ...  30  |
+---------+-----------+--------------+
| ankle_2 | (1, 1, 0) | -70 ... -30  |
+---------+-----------+--------------+
| hip_3   | (0, 0, 1) | -30 ...  30  |
+---------+-----------+--------------+
| ankle_3 | (1, 1, 0) | -70 ... -30  |
+---------+-----------+--------------+
| hip_4   | (0, 0, 1) | -30 ...  30  |
+---------+-----------+--------------+
| ankle_4 | (1, 1, 0) |  30 ...  70  |
+---------+-----------+--------------+

State information for the above joints is received via a :ref:`Joint State Perceptor <agent-protocol_joint-state-perceptor>`.

Joint actions are performed using :ref:`Motor Effectors <agent-protocol_motor-effector>` for the respective joints.

.. _robot-model_ant-sensors:

Sensors
-------

The Ant robot is equipped with the following sensors:

+---------------------------------------------------------------+------------+--------------+---------------+
| Perceptor                                                     | Name       | Frame / Body | Position      |
+===============================================================+============+==============+===============+
| :ref:`Position <agent-protocol_position-perceptor>`           | torso_pos  | torso        | origin        |
+---------------------------------------------------------------+------------+--------------+---------------+
| :ref:`Orientation <agent-protocol_orientation-perceptor>`     | torso_quat | torso        | origin        |
+---------------------------------------------------------------+------------+--------------+---------------+
| :ref:`Gyro\-Rate <agent-protocol_gyro-rate-perceptor>`        | torso_gyro | torso        | origin        |
+---------------------------------------------------------------+------------+--------------+---------------+
| :ref:`Accelerometer <agent-protocol_accelerometer-perceptor>` | toros_acc  | torso        | origin        |
+---------------------------------------------------------------+------------+--------------+---------------+
| :ref:`Ideal Vision <agent-protocol_vision-perceptor>`         | See        | torso        | (0.2, 0, 0.1) |
+---------------------------------------------------------------+------------+--------------+---------------+
