.. _dev:

Development
===========

This chapter describes how to get started with developing the simulator.


Setup / Installation
--------------------

This project uses `hatch <https://hatch.pypa.io>`_ as project management tool.  
You can install *hatch* via your package manager or ``pip install hatch`` (or ``pipx install hatch`` on managed systems).

*Hatch* is managing various virtual environments in the background, which are used to run scripts, tests, etc.
You don't need to manage / create any virtual environment by yourself.
Virtual environments are automatically created by hatch on demand (when running scripts).
The project itself and the environment specific dependencies specified in the *pyproject.toml* file are automatically installed in these virtual environments.
Check out the `Docs <https://hatch.pypa.io/dev/tutorials/environment/basic-usage/>`_ for more information.


Hatch Configuration
^^^^^^^^^^^^^^^^^^^

In general, there is no extra configuration required to get running.
However, especially when running from the integrated terminal of an IDE, you might want to change the default location where *hatch* places its virtual environments.

The virtual environment base path is specified by the "dirs.data" configuration entry.
You can inspect your current config with the following command:

.. code:: bash

    hatch config show

which should give you an output similar to:

.. code:: bash

    ...

    [dirs]
    project = []
    python = "isolated"
    data = "/some/path/on/your/system"
    cache = "/home/user/.cache/hatch"

    ...

You can set the "dirs.data" entry to an arbitrary directory (e.g. "/home/user/venvs/hatch" below) by running the following command:

.. code:: bash

    hatch config set dirs.data /home/user/venvs/hatch


Virtual Environments
^^^^^^^^^^^^^^^^^^^^

Note: This step is not mandatory.
As mentioned before, *hatch* will automatically manage virtual environments for you.

But, even though virtual environments are created on demand, you can still manage them explicitly.
E.g. you can create the default virtual environment(s) by running:

.. code:: bash

    hatch env create

in the project directory (containing the *pyproject.toml* file).
*Hatch* will create the default virtual environment and install the package in dev-mode as well as its dependencies.

The commands ``hatch env show`` and ``hatch env find`` can help to get an understanding which virtual environments exist and where they are located.


Run the Server
--------------

The main scripts of the package are specified in the *pyproject.toml* file.
The simulation server script is called ``mjrcssserver`` and can be executed using *hatch*:

.. code:: bash

    hatch run mjrcssserver

With this command, *hatch* will run the ``mjrcssserver`` command within the default virtual environment.
Use the ``-h`` option to get some help for the command (``hatch run mjrcssserver -h``).


Development Scripts
-------------------

Build
^^^^^

In order to build the project, simply run:

.. code:: bash

    hatch build

in the root directory of the project.
This will generate a "dist" directory, in which you'll find the source release and wheel.

Use ``hatch clean`` to clean the "dist" directory again.


Type Checking
^^^^^^^^^^^^^

This project heavily relies on type hints.
You can run a MyPy type checking with the following command:

.. code:: bash

    hatch run types:check

Which will effectively execute the "check" command, defined in the *pyproject.toml* file in the "types" environment (in which MyPy is installed).


Formatting
^^^^^^^^^^

You can run the ruff code formatter via:

.. code:: bash

    hatch fmt

to format the code and get some suggestions for improving your code.


Generate Docs
^^^^^^^^^^^^^

The project uses sphinx for documentation.
Running the following command will generate the html project docs in the "docs/build/html" directory:

.. code:: bash

    hatch run doc:html

Clean the docs build directory via ``hatch run doc:clean``.


Run Tests
^^^^^^^^^

.. code:: bash

    hatch test

will run the tests associated with the project (once there are any...).
