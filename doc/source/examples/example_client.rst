.. _example-client:

Example Client
==============

The simulation server repository contains a simple example client that demonstrates the fundamental steps required to spawn a robot and to communicate with the simulation server.

Example client: `example/client/ <https://gitlab.com/robocup-sim/rcssservermj/-/tree/master/example/client/>`_


.. _example-client_setup:

Setup / Prerequisites
---------------------

In order to run the example client, you first need to setup a virtual environment with the required dependencies.

The example client script only depends on ``numpy``.
As such, any Python environment with *numpy* installed should work.


.. _example-client_start:

Start a Client
--------------

Navigate to the :file:`example/client` directory and execute the :file:`mujoco_client.py` script:

.. code:: bash

    python3 mujoco_client.py -s localhost -p 60000 -t test -n 1


.. _example-client_cli-parameter:

CLI Parameter
^^^^^^^^^^^^^

-s ip             The server IP (default: 'localhost').
-p agent_port     The agent port (default: 60000).
-t team_name      The team name (default: 'Test').
-n player_number  The player number (default: 1).
-r robot_model    The robot model to spawn (default: 'ant', referring to the :ref:`Ant Robot <robot-model_ant>`). See :ref:`robot models <robot-models>` for available options.


.. _example-client_stop:

Stop a Running Client
---------------------

Simply :kbd:`ctrl+c` the client process to disconnect a client again.
