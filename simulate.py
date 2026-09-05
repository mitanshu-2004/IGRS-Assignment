import math
import time

import pybullet as p

from puma import Puma560, quintic_path, solve, unsafe_between

TIMESTEP = 1.0 / 240.0

PEAK_SPEED = math.radians(90)
MIN_MOVE_TIME = 1.0

HOME = [0.0, math.radians(20), math.radians(25), 0.0, math.radians(-20), 0.0]


def start_clients():
    gui = p.connect(p.GUI)
    check = p.connect(p.DIRECT)

    p.setTimeStep(TIMESTEP, physicsClientId=gui)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=gui)
    p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=50, cameraPitch=-18,
                                 cameraTargetPosition=[0.15, 0, 0.6], physicsClientId=gui)
    draw_ground(gui)
    return gui, check


def draw_ground(gui):
    dots = []
    for step in range(-6, 7):
        edge = step * 0.15
        for n in range(91):
            along = -0.9 + 0.02 * n
            dots += [[edge, along, 0], [along, edge, 0]]
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=gui)
    p.addUserDebugPoints(dots, [[0.55, 0.55, 0.6]] * len(dots), 2, physicsClientId=gui)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=gui)


def make_marker(gui):
    shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.035, rgbaColor=[0, 0, 0, 0],
                                physicsClientId=gui)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape, physicsClientId=gui)


# removeAllUserDebugItems leaves debug points on screen.
_drawn = []


def show(gui, robot, marker, target, colour, path=None, reason=None):
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=gui)
    try:
        while _drawn:
            p.removeUserDebugItem(_drawn.pop(), physicsClientId=gui)
        p.changeVisualShape(marker, -1, rgbaColor=colour + [0 if target is None else 1],
                            physicsClientId=gui)
        if target is None:
            return
        p.resetBasePositionAndOrientation(marker, list(target), [0, 0, 0, 1], physicsClientId=gui)
        if path:
            points = [robot.tool_position(angles).tolist() for angles in path[::5]]
            _drawn.append(p.addUserDebugPoints(points, [[1.0, 0.45, 0.0]] * len(points), pointSize=6,
                                               physicsClientId=gui))
        if reason:
            _drawn.append(p.addUserDebugText(reason, [0, 0, 1.45],
                                             textColorRGB=colour, textSize=1.3, physicsClientId=gui))
    finally:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=gui)


def drive(robot, path, speeds, gui):
    # Without targetVelocities Bullet brakes toward zero velocity every step.
    start = time.perf_counter()
    steps = 0
    for angles, speed in zip(path, speeds):
        p.setJointMotorControlArray(robot.gui_body, robot.joints, p.POSITION_CONTROL,
                                    targetPositions=list(angles), targetVelocities=list(speed),
                                    physicsClientId=gui)
        p.stepSimulation(physicsClientId=gui)
        steps += 1
        time.sleep(max(0.0, start + steps * TIMESTEP - time.perf_counter()))

        touching = robot.self_collisions(robot.current_angles())
        if touching:
            print(f"\n  Stopped: the arm has touched itself, {robot.describe_collisions(touching)}.")
            return False
    return True


def path_to(current, goal):
    travel = max(abs(g - now) for now, g in zip(current, goal))
    duration = max(MIN_MOVE_TIME, 1.875 * travel / PEAK_SPEED)
    return quintic_path(current, goal, duration, round(duration / TIMESTEP))


def go_to(robot, target, marker, gui):
    if robot.self_collisions(robot.current_angles()):
        print("\n  The arm is stopped in contact with itself and cannot plan its way out of that.")
        print("    Type h to return it to the starting pose.")
        return

    answer = solve(robot, target)
    if answer.reason:
        print(f"\n  Cannot move there: {answer.reason}.")
        for line in answer.detail:
            print(f"    {line}")
        show(gui, robot, marker, target, [1, 0.2, 0.4], reason=answer.label)
        return

    current = robot.current_angles()
    unsafe = unsafe_between(robot, current, answer.angles)
    if unsafe:
        fraction, pairs = unsafe
        print("\n  Cannot move there: the arm would collide with itself on the way,")
        print(f"    {pairs}, about {100 * fraction:.0f}% along the path.")
        print("    Both ends of the move are fine, the motion between them is not.")
        show(gui, robot, marker, target, [1, 0.5, 0.1], reason="collides on the way")
        return

    print(f"\n  Target accepted, {answer.error * 1000:.1f} mm from the requested point.")
    print("    Joint angles (deg): " + "  ".join(f"{name}={math.degrees(angle):+.1f}"
                                                 for name, angle in zip(robot.names, answer.angles)))
    path, speeds = path_to(current, answer.angles)
    show(gui, robot, marker, target, [0, 0.9, 1], path)

    if drive(robot, path, speeds, gui):
        landed = robot.tool_position()
        print(f"    Tool is at ({landed[0]:+.3f}, {landed[1]:+.3f}, {landed[2]:+.3f}).")


def place_home(robot, gui):
    for joint, angle in zip(robot.joints, HOME):
        p.resetJointState(robot.gui_body, joint, angle, physicsClientId=gui)
    # resetJointState leaves the previous motor targets in place.
    p.setJointMotorControlArray(robot.gui_body, robot.joints, p.POSITION_CONTROL,
                                targetPositions=HOME, physicsClientId=gui)
    for _ in range(60):
        p.stepSimulation(physicsClientId=gui)


def go_home(robot, gui, marker):
    show(gui, robot, marker, None, [0, 0, 0])
    current = robot.current_angles()
    if robot.self_collisions(current) or unsafe_between(robot, current, HOME):
        place_home(robot, gui)
        print("\n  Placed at the starting pose, since the arm could not drive there without touching itself.")
        return
    if drive(robot, *path_to(current, HOME), gui):
        print("    Back at the starting pose.")


def read_target():
    while True:
        raw = input("\nTarget X Y Z in metres, h for home, q to quit: ").strip().lower()
        if raw in ("q", "quit", "exit"):
            return None
        if raw in ("h", "home"):
            return "home"
        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            print("  Enter three numbers, for example: 0.45 -0.15 0.30")
            continue
        try:
            target = [float(part) for part in parts]
        except ValueError:
            print("  Those are not three numbers.")
            continue
        if not all(-1e3 <= value <= 1e3 for value in target):
            print("  Targets are ordinary numbers, in metres from the base.")
            continue
        return target


def main():
    gui, check = start_clients()
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=gui)
        robot = Puma560(gui, check)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=gui)
        marker = make_marker(gui)
        place_home(robot, gui)

        tool = robot.tool_position()
        print(f"PUMA 560 loaded. {len(robot.joints)} joints, reach at most "
              f"{robot.max_reach:.3f} m from the shoulder at z={robot.shoulder[2]:.3f}.")
        print(f"Tool starts at ({tool[0]:+.3f}, {tool[1]:+.3f}, {tool[2]:+.3f}).")

        while True:
            target = read_target()
            if target is None:
                break
            if target == "home":
                go_home(robot, gui, marker)
                continue
            go_to(robot, target, marker, gui)
    except (EOFError, KeyboardInterrupt):
        print("\n  Interrupted.")
    finally:
        p.disconnect(physicsClientId=gui)
        p.disconnect(physicsClientId=check)


if __name__ == "__main__":
    main()
