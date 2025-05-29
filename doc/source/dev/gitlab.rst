GitLab
======

This chapter describes GitLab setup and integrations.


Project Container Registry
--------------------------

To reduce server load and network traffic for GitLab, we use our own docker images for running the CI/CD pipelines.

Container / Images
^^^^^^^^^^^^^^^^^^

As the project is based on python and managed with hatch, the docker image can be any OS providing these dependencies.
The following example *Dockerfile* is based on *Ubuntu 24.04*.

.. code-block::
    :caption: Dockerfile using Ubuntu 24.04 with Python, pipx and Hatch

    FROM ubuntu:24.04

    ENV TZ=Europe/Berlin
    ENV PATH="$PATH:$HOME/.local/bin"
    RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

    RUN apt-get update \
        && apt-get install -y --no-install-recommends python3 pipx \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*

    RUN pipx install hatch


Image Publishing
^^^^^^^^^^^^^^^^

With the above Dockerfile, you can execute the following commands to create a local image and subsequently publish it to the project container registry.

Build the docker image in the current directory:

.. code:: bash

    docker build -t registry.gitlab.com/robocup-sim/rcssservermj/uhatch:24.04 .

The tag name is relevant for publishing and referencing the image later in CI/CD.
The above command will create an image called "uhatch" with version information "24.04" in the "robocup-sim/rcssservermj" container repository.

Before publishing the image, you first need to login to the GitLab registry:

.. code:: bash

    docker login registry.gitlab.com

Once logged in, you can push the image to the repository:

.. code:: bash

    docker push registry.gitlab.com/robocup-sim/rcssservermj/uhatch:24.04


Image Usage
^^^^^^^^^^^

In the ".gitlab-ci.yml" file, you can reference the project images using the ``$CI_REGISTRY`` variable.

For using the image from the example above, use:

.. code::

    $CI_REGISTRY/robocup-sim/rcssservermj/uhatch:24.04
