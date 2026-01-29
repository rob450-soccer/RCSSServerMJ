# RCSSServerMJ

The RoboCup Soccer Simulation Server based on the MuJoCo physics engine.

![Soccer Simulation Screenshot](https://gitlab.com/robocup-sim/rcssservermj/-/raw/master/doc/source/_static/img/screenshot-T1.png)

__Video__: [https://youtu.be/iYWw7vATQU4](https://youtu.be/iYWw7vATQU4)

## Documentation

The package documentation and user guide can be found at: [robocup-sim.gitlab.io/rcssservermj](https://robocup-sim.gitlab.io/rcssservermj/).

## Installation

The simulation server (**rcsssmj** package) can be installed from [PyPI](https://pypi.org/project/rcsssmj/) via:

```bash
pip install rcsssmj
```

For more detailed installation instructions (virtual environments, installation from source, development setup, etc.) please refer to the [Installation](https://robocup-sim.gitlab.io/rcssservermj/user/installation.html) chapter of the [User Guide](https://robocup-sim.gitlab.io/rcssservermj/user/index.html).

## Instructions

Start the soccer simulation server:

```bash
rcssservermj -a 127.0.0.1 -p 60000 -m 60001
```

CLI parameter:

- `-a <ip>` to specify the server IP (default: 'localhost')
- `-p <agent_port>` to specify the agent port (default: 60000)
- `-m <monitor_port>` to specify the monitor port (default: 60001)

Stop the server: Simply <kbd>ctrl+c</kbd> the server process.

For more detailed instructions on how to run a simulation (including the full list of CLI parameter) please refer to the [Running a Simulation](https://robocup-sim.gitlab.io/rcssservermj/user/running_a_simulation.html) chapter of the [User Guide](https://robocup-sim.gitlab.io/rcssservermj/user/index.html).

## Bug Reports and Feature Requests

Please use GitLab [Issues](https://gitlab.com/robocup-sim/rcssservermj/-/issues) for reporting bugs, requesting new features and discussing development-related subjects.

## Credits

- Pitch texture taken from [dm_control](https://github.com/google-deepmind/dm_control/blob/main/dm_control/locomotion/soccer/assets/pitch/pitch_nologo_l.png) (Apache 2.0 license)
- Ant robot XML taken from [gymnasium](https://github.com/Farama-Foundation/Gymnasium/blob/main/gymnasium/envs/mujoco/assets/ant.xml) (MIT License)
- T1 robot XML and assets adapted / taken from [booster_gym](https://github.com/BoosterRobotics/booster_gym/blob/main/resources/T1/T1_serial.xml) (Apache 2.0 License)
- K1 robot XML and assets adapted / taken from [booster_assets](https://github.com/BoosterRobotics/booster_assets/blob/main/robots/K1/K1_22dof.xml) (BSD 3-Clause License)

## License and Disclaimer

Copyright (c) 2025 The RCSSServerMJ authors.

This project is licensed under the terms of the [MIT License](https://gitlab.com/robocup-sim/rcssservermj/-/raw/master/LICENSE).  
This applies to all source files in this repository except where otherwise noted.

ReStructuredText documents, images, and videos in the doc directory are made available under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0) license.
You may obtain a copy of the License at [https://creativecommons.org/licenses/by/4.0/legalcode](https://creativecommons.org/licenses/by/4.0/legalcode).
