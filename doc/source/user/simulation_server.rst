.. _sim-server:

Simulation Server
=================

The simulation server (:py:class:`SimServer <rcsssmj.simulation.SimServer>`) is responsible for running the main simulation loop.
It periodically sends sensor messages to connected agents and collects corresponding actions in each simulation cycle.
Similarly, it sends the simulation state to connected monitors and forwards monitor commands to the game director (referee).

Game related aspects of the simulation are handled by a specific game director (referee), allowing flexible rule books.

The simulation server exposes two network interfaces, one for external agent processes and one for external monitor processes.
This setup allows to split simulation, agents and monitors into separate processes, potentially running on different machines.

TODO: Create an overview image showing server, agents and monitor(s).


.. _sim-server_network-protocol:

Network Protocol
----------------

Communication between simulation server and clients (agents / monitors) is realized via a simple TCP/IP based message exchange.
Each message consists of a length prefix and a message payload.
The length prefix is a 32 bit unsigned integer in network order, i.e. big endian notation with the most significant bits transferred first.
The message payload reflects the agent / monitor protocols specified in :ref:`Agent Protocol <agent-protocol>` and :ref:`Monitor Protocol <monitor-protocol>` and is further encoded using a message format specified below.

.. code-block::
    :caption: Network Message Protocol

    +--------+--------+--------+--------+--------+---- ... ----+--------+
    |       length prefix: 'L'          |        message payload        |
    +--------+--------+--------+--------+--------+---- ... ----+--------+
    0        1        2        3        4        5     ...     3+L      4+L


.. _sim-server_message-formats:

Message Formats
^^^^^^^^^^^^^^^

When exchanging messages between the simulation server and clients (agents / monitors), the message content is encoded using a specific message format.

Currently, the simulation server only provides an S-Expression based message format.
However, future versions of the server might implement additional message formats (e.g. a binary format), reducing computation and network load.


.. _sim-server_s-expression-format:

S-Expression Format
"""""""""""""""""""

This message protocol is based on an flavor of `symbolic expressions <https://en.wikipedia.org/wiki/S-expression>`_.

- ASCII based
- flexible
- human readable
- some overhead


.. _sim-server_agent-communication:

Server\-Agent Communication
---------------------------

The simulation server exposes a network interface to all agents on TCP port 60000 by default.

When an agent connects to the server, the agent must first send an :ref:`initialization request <agent-protocol_init-request>` message.
After successfull initialization, the server continuously sends perception messages (one message per simulation cycle) to the agent representing the output of the agent's perceptors.
Detailed information about the perceptor message format can be found in the :ref:`Agent Protocol <agent-protocol>` section.

In response to these perceptor messages, the agent may manupulate the state of its robot model within the simulation by sending effector messages.
Effectors are translated to actuator commands (e.g. motor actions) by the simulation server and applied in the subsequent simulation cycle.
Detailed information about the effector message format can again be found in the :ref:`Agent Protocol <agent-protocol>` section.

.. important::

    **Combine all effector actions for a simulation cycle in a single response message!**

    While an agent can in theory send multiple action messages for a single simulation cycle, it is strongly adviced to combine and send all effector actions in a single response message.
    This is because of the sync-mode of the server.
    If the server runs in sync-mode, it will wait for exactly one response message from each connected agent before simulating the next cycle.
    Thus, sending multiple response messages in a single simulation cycle will likely result in desynchronization between server and agent.


.. _sim-server_monitor-communication:

Server\-Monitor Communication
-----------------------------

The simulation server exposes a network interface to all monitors on TCP port 60001 by default.

This interface allows external processes to be periodically notified of the simulation's state for purposes of visualisation, logging, etc.
In addition, the connected monitors may send various commands to the simulation server, acting as an external game referee.

TODO: describe communication sequence
