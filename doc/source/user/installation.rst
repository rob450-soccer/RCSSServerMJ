.. _install:

Installation
============

The RCSSServerMJ (**rcsssmj** package) is currently only published on `PyPI <https://pypi.org/project/rcsssmj/>`_.

Publishing on *conda-forge* might be added in the future.
Until then, you need to use ``pip`` to install the simulation server package into an anaconda managed environment.


.. _install_pypi:

Installation from PyPI
----------------------

You can use ``pipx`` or ``pip`` to install the simulation server from `PyPI <https://pypi.org/project/rcsssmj/>`_.


.. _install_pipx:

pipx (managed systems)
^^^^^^^^^^^^^^^^^^^^^^

`pipx <https://pipx.pypa.io>`_ will automatically create a dedicated virtual environment and install the simulation server into this environment.
The ``rcssservermj`` command is exposed to the user shell and can be run from any location.
This is the go-to option on managed systems when you just want to run a release version of the simulation server.

.. code:: bash

    pipx install rcsssmj


.. _install_pip:

pip (virtual environments)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Create and activate a new virtual environment for installing the simulation server.
While this step is optional, it is considered good practice, as it prevents pollution of your system installation (especially on managed systems).

.. code:: bash

    cd path/to/some/directory
    python3 -m venv rcsssmj
    source rcsssmj/bin/activate

.. note::
    
    Virtual environments managed by Anaconda work fine, too.

Once the new virtual environment has been created and activated, you can install the simulation server with the following command:

.. code:: bash

    python3 -m pip install rcsssmj


.. _install_source:

Installation from Source
------------------------

Obtain a `copy of the source code <https://gitlab.com/robocup-sim/rcssservermj/-/releases>`_ or clone the `repository <https://gitlab.com/robocup-sim/rcssservermj>`_.

Create and activate a new virtual environment for installing the simulation server.

.. code:: bash

    cd path/to/some/directory
    python3 -m venv rcsssmj
    source rcsssmj/bin/activate

Navigate to your local source code copy / repository location and install the simulation server dependencies and package:

.. code:: bash

    cd path/to/rcsssmj-repo
    python3 -m pip install -r requirements.txt
    python3 -m pip install .


.. _install_dev:

Installation for (Local) Development
------------------------------------

This project uses `hatch <https://hatch.pypa.io>`_ as project management tool.
Please refer to the :ref:`Development Setup / Installation <dev_setup>` chapter for more information on how to setup the project for local development.

Alternatively, you can always use the :ref:`Installation from Source <install_source>` approach for installing the package in any python environment.
Make sure to use the ``-e`` flag for installing the package in editable mode.
