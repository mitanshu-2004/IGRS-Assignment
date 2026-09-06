# PUMA 560 motion simulation in PyBullet

Type a target point and the arm moves to it, or explains why it cannot.

The simulation uses Python, PyBullet and NumPy. It does not use ROS, ROS 2, MoveIt, RViz or Gazebo.

## Running it

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulate.py
```

PyBullet may build from source on Python 3.12 or later, which can take a few minutes and requires a C++ compiler.

A PyBullet window opens with the arm at its starting pose. Enter target coordinates through the terminal:

```text
Target X Y Z in metres, h for home, q to quit: 0.45 -0.15 0.30
```

Coordinates are in metres in the world frame. The robot base is at the origin and its shoulder is 0.672 m above it.

Some example targets are:

| Target | Result |
| --- | --- |
| `0.45 -0.15 0.30` | Accepted |
| `0.9 0.27 0.2` | Outside the arm's reach |
| `0.01 0.01 0.2` | No legal pose found |
| `0.36 -0.09 1.16` | Mostly rejected by joint limits |
| `0.1 0.03 0.21` | Rejected by collision or joint-limit checks |

To reproduce a path-collision refusal, enter these targets in order:

```text
0.75 -0.54 0.62
0.10 -0.21 0.00
-0.24 0.43 0.34
```

The first two movements are accepted. The third is refused because the arm would collide with itself during the movement.

Accepted targets are shown in cyan with the planned tool path. Refused targets are shown in red, or orange when the target pose is valid but the path is unsafe. The reason is also printed in the terminal.

Enter `h` to return the arm to its starting pose. The arm normally follows a planned path home. If it is already in contact or cannot return safely, it is placed directly at the home configuration.

## URDF collision geometry

The supplied URDF contained visual geometry but no collision geometry. Without `<collision>` elements, PyBullet could not detect collisions between the robot's links.

A collision element matching the visual mesh and origin was added to each of the seven links. The existing material opacity was also changed from `0.8` to `1` so the links render as solid objects.

## How it works

`simulate.py` handles the GUI and user input. `puma.py` contains the robot model, inverse kinematics, trajectory generation and safety checks. `measure.py` checks the arm-radius value used for path sampling.

For each target, the program:

1. Rejects points outside the arm's maximum reach.
2. Runs inverse kinematics from several deterministic starting poses.
3. Uses forward kinematics to verify that the tool reaches the target within 1 mm.
4. Rejects solutions that exceed joint limits or cause self-collision.
5. Checks the complete joint-space path for self-collision.
6. Executes an accepted path using a smooth quintic trajectory.

The movement duration keeps the peak joint speed below 90 degrees per second. During movement, the robot's actual configuration is checked after every simulation step, and the motion stops if a self-collision is detected.
