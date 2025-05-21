# RCSSServerMJ

The RoboCup Soccer Simulation Server based on the MuJoCo physics engine.

![Simulation Screenshot](screenshot.png)

Video: [https://youtu.be/iYWw7vATQU4](https://youtu.be/iYWw7vATQU4)

## Installation

### From Package Repositories

### Development Installation

This project uses [hatch](https://hatch.pypa.io) as project management tool.  
You can install _hatch_ via your package manager or `pip install hatch` (or `pipx install hatch` on managed systems).

After installing _hatch_, navigate to your local repository and run arbitrary scripts in isolated virtual environments via the _hatch run_ command (e.g. `hatch run mjrcssserver` for running the server with default parameter).

### Local Installation

Navigate to your local repository and install the simulation server dependencies and package:

```bash
cd path/to/repo
pip install -r requirements.txt
pip install .
```

Use `-e` option to install the package in editing / development mode (e.g. `pip install -e .`).

## Instructions

### Server

Start the server:

```bash
mjrcssserver -s 127.0.0.1 -p 60000 -m 60001
```

CLI parameter:

- `-s <ip>` to specify the server IP (default: 'localhost')
- `-p <client_port>` to specify the client port (default: 60000)
- `-m <monitor_port>` to specify the monitor port (default: 60001)

Stop the server: Simply <kbd>ctrl+c</kbd> the server process.

### Example client

#### Start clients one by one

Start a client:

```bash
python mujoco_client.py -s localhost -p 60000 -t test -n 1
```

CLI parameter:

- `-s <ip>` to specify the server IP (default: 'localhost')
- `-p <client_port>` to specify the client port (default: 60000)
- `-m <monitor_port>` to specify the team name (default: 'Test')
- `-n <player_number>` to specify the player number (default: 1)

Disconnect a client: Simply <kbd>ctrl+c</kbd> the client process.

#### Start full teams

Start the first team:

```bash
start_team.sh team0
```

Start the second team:

```bash
start_team.sh team1
```

## Credit

- Pitch texture taken from [dm_control](https://github.com/google-deepmind/dm_control/blob/main/dm_control/locomotion/soccer/assets/pitch/pitch_nologo_l.png)
- Ant XML taken from [gymnasium](https://github.com/Farama-Foundation/Gymnasium/blob/main/gymnasium/envs/mujoco/assets/ant.xml)
