.. _robot-models:

Robot Models
============

Robot Models refer to the physical representation of agents within the simulation.

The simulator ships with a collection of robot models that can be selected by a client.
Upon connecting, clients have to specify the robot model representation of the corresponding agent in the simulation as part of the :ref:`Init Requst <agent-protocol_init-request>`.

Robot model files are located in the '*resources/robots*' directory.
Each robot model is organized in a separate subdirectory named after the model.

.. toctree::
    :maxdepth: 1
    :caption: General

    Ant <ant>

.. toctree::
    :maxdepth: 1
    :caption: Humanoid

    T1 <T1>
    K1 <K1>
