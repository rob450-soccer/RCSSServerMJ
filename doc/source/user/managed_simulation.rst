.. _managed-sim:

Managed Simulation
==================

The managed simulation is specifically tailored towards machine learning environments.
Instead of exposing network interfaces for communication with external processes, the managed simulation aims to integrate the learning (as well as additional) agents into the simulation in a synchronous way, allowing to keep everything within a single training process.

The progress of the simulation is externally managed by the specific learning environment.
Furthermore, the learning environment has full access to the simulation state and agent perceptors and exhibits full control over the agent's actions.


.. _managed-sim_sim-control:

Simulation Control
------------------

TODO: describe how the three main methods of the managed simulation are intended to be used


.. _managed-sim_gym:

Gymnasium Integration
---------------------

TODO: describe how the use the managed simulation in a gymnasium environment
