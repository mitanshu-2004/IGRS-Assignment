# PUMA 560 motion simulation in PyBullet

Type a target point and the arm moves to it, or explains why it can't.

Python and PyBullet, with numpy for the arithmetic. No ROS, ROS 2, MoveIt, RViz or Gazebo.

## Running it

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python simulate.py
```

PyBullet ships no wheel for Python 3.12 or later, so pip builds it from source: allow a few minutes and a C++ compiler.

A PyBullet window opens with the arm at its starting pose. Targets go in at the terminal:

```
Target X Y Z in metres, h for home, q to quit: 0.45 -0.15 0.30
```

Coordinates are metres in the world frame, base at the origin, shoulder 0.672 m up. These cover the answers you are likely to see:

| target | result |
| --- | --- |
| `0.45 -0.15 0.30` | accepted |
| `0.9 0.27 0.2` | beyond the arm's reach |
| `0.01 0.01 0.2` | inside the reach, but the search finds no pose for it |
| `0.36 -0.09 1.16` | mostly joint limits, a few attempts fall short |
| `0.1 0.03 0.21` | a mix: some fold the arm into itself, some break a limit |

The third kind of refusal needs a sequence, because both endpoints have to be fine and only the motion between them bad: `0.75 -0.54 0.62`, then `0.10 -0.21 0.00`, then `-0.24 0.43 0.34`, which is refused a few percent along the path. It is rare.

An accepted target is drawn cyan with the planned tool path; a refused one red, or orange when both ends are fine and only the path between them is not. The reason is written above the arm in the window and in full on the terminal. `h` drives the arm back to its starting pose like any other move. If the arm is stopped in contact with itself, or the way back would touch, it is placed there instead: the one move that can teleport, and the only way out of a contact.

## The URDF needed collision geometry

The file I was sent describes each link with a `<visual>` element and nothing else, and PyBullet builds collision shapes from `<collision>` elements. So every link loaded with zero collision shapes, and asking Bullet for contacts between the robot and itself returned nothing even with the arm folded hard into itself. `URDF_USE_SELF_COLLISION` does not help: it only stops Bullet skipping a body's own links, and there was nothing there to test.

Each of the seven links now has a `<collision>` repeating the `<geometry>` and `<origin>` of its `<visual>`. The origin matters as much as the mesh: link5's sits 0.362 m up its own frame and turned 90 degrees, and without it the collision shape would sit a third of a metre from the link. The material alpha went from 0.8 to 1 as well, because the links rendered see-through. There is a note at the top of the file recording both. Nothing else changed.

## How it works

`simulate.py` is the window and the input loop, `puma.py` is the robot, the checks and the solver, and `measure.py` backs one number in it.

A target further from the shoulder than the joint offsets can sum to is refused as out of reach. Anything else goes to Bullet's inverse kinematics from a hundred and one starting poses, the mid-range pose first and then random legal ones from a fixed seed, so the same point always gives the same answer. Bullet hands back its closest attempt whether or not it converged, so each answer is run through forward kinematics and must land within a millimetre. Joint limits are checked on the answer rather than passed to the solver, since passing them switches it into a null-space mode that misses by centimetres on about a third of targets and still returns out-of-range angles. A pose that passes both is checked for self-collision, and the first one clear of all three is the goal. If none is, the refusal counts how the attempts failed, with one example of each, because naming one reason for a hundred starting poses would be arbitrary.

With a goal, the straight line in joint space from the current pose is sampled so that nothing on the arm moves more than 2 mm between samples, and every sample is collision-checked. No point the arm carries is more than a metre from the axis turning it, so joint travel bounds how far anything moves; `measure.py` prints the measurement behind that metre. The 2 mm is Bullet's own collision margin, 1 mm per hull: a pose the check passes has at least that much real clearance, so a touch cannot hide between two clear samples. If a sample touches, the move is refused with where along the path and which links.

The move scales the joint angles by s(t) = 10t³ − 15t⁴ + 6t⁵, so it leaves and returns to rest smoothly and every pose on the way is one the arm can hold. It lasts as long as its largest joint travel needs to stay under 90 deg/s, one second at least, so a short reach and a full unwind of the base both look deliberate. The waypoints go to the position motors with their velocities, the engine steps between them, and after every step the pose the arm is actually in is checked again. It stops if it touches itself.
