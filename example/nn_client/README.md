# Neural Network Client Example

This code example demonstrates how to create a simple client that loads a pre-trained neural network model and uses it for locomotion control.

The neural network is trained with the dedicated RoboCup training environment in the [RL-X](https://github.com/nico-bohlinger/RL-X/tree/master/rl_x/environments/custom_mujoco/robocup_soccer) Reinforcement Learning framework.

## Usage
1. Install the required dependencies:
```bash
pip install -r requirements.txt
```

2. Run the simulation server (make sure to have the RCSSServerMJ installed and running):
```bash
python rcssservermj --field sim3d_7vs7
```
or, if you have cloned and installed the RCSSServerMJ from source:
```bash
python src/rcsssmj/__main__.py --field sim3d_7vs7
```

3. In another terminal, start a full team of 11 agents:
```bash
bash start_team.sh team1 T1
```

4. In another terminal, start a second team of 11 agents:
```bash
bash start_team.sh team2 T1
```

If everything works, you should see a game similar to the one shown in this [video](https://youtu.be/f6MvmqiiU6Q)