import contextlib
import itertools
import os
import sys

from typing import NamedTuple

import numpy as np
import pybullet as p

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "puma560_description", "urdf", "puma560_robot.urdf")

POSITION_TOLERANCE = 0.001
IK_DAMPING = 1e-3
IK_RESTARTS = 100
ARM_RADIUS = 1.0
STEP_CLEARANCE = 0.002


# Bullet prints a missing-inertial warning per link straight to fd 1.
@contextlib.contextmanager
def _quiet_stdout():
    sys.stdout.flush()
    saved, null = os.dup(1), os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(null, 1)
        yield
    finally:
        os.dup2(saved, 1)
        os.close(null)
        os.close(saved)


class Puma560:
    def __init__(self, gui_client, check_client):
        self.gui_client = gui_client
        self.check_client = check_client

        flags = p.URDF_USE_SELF_COLLISION
        with _quiet_stdout():
            self.gui_body = p.loadURDF(URDF_PATH, useFixedBase=True, flags=flags,
                                       physicsClientId=gui_client)
            self.check_body = p.loadURDF(URDF_PATH, useFixedBase=True, flags=flags,
                                         physicsClientId=check_client)

        self.joints = list(range(p.getNumJoints(self.gui_body, physicsClientId=gui_client)))
        self.end_effector = self.joints[-1]
        self.base_name = p.getBodyInfo(self.gui_body, physicsClientId=gui_client)[0].decode()

        self.names, self.lower, self.upper = [], [], []
        parent, origins = {}, []
        for joint in self.joints:
            info = p.getJointInfo(self.gui_body, joint, physicsClientId=gui_client)
            self.names.append(info[1].decode())
            self.lower.append(info[8])
            self.upper.append(info[9])
            parent[joint] = info[16]
            origins.append(np.array(info[14]))

        self.midpoint = [(lo + hi) / 2 for lo, hi in zip(self.lower, self.upper)]

        # Adjacent links touch at their joint in every pose.
        joined = {frozenset((joint, parent[joint])) for joint in self.joints}
        self.pairs = [(a, b) for a, b in itertools.combinations(range(-1, len(self.joints)), 2)
                      if frozenset((a, b)) not in joined]

        self.shoulder = origins[0]
        self.max_reach = float(sum(np.linalg.norm(o) for o in origins[1:]))

    def set_check_pose(self, angles):
        for joint, angle in zip(self.joints, angles):
            p.resetJointState(self.check_body, joint, angle, physicsClientId=self.check_client)

    def tool_position(self, angles=None):
        if angles is None:
            body, client = self.gui_body, self.gui_client
        else:
            self.set_check_pose(angles)
            body, client = self.check_body, self.check_client

        state = p.getLinkState(body, self.end_effector, computeForwardKinematics=True, physicsClientId=client)
        return np.array(state[4])

    def current_angles(self):
        return [state[0] for state in p.getJointStates(self.gui_body, self.joints, physicsClientId=self.gui_client)]

    def joint_limit_violations(self, angles):
        violations = []
        for i, angle in enumerate(angles):
            if not self.lower[i] <= angle <= self.upper[i]:
                violations.append(f"{self.names[i]} would need {np.degrees(angle):+.1f} deg, outside its range "
                                  f"[{np.degrees(self.lower[i]):+.1f}, {np.degrees(self.upper[i]):+.1f}] deg")
        return violations

    def self_collisions(self, angles):
        self.set_check_pose(angles)
        return {(a, b) for a, b in self.pairs
                if p.getClosestPoints(bodyA=self.check_body, bodyB=self.check_body, distance=0.0,
                                      linkIndexA=a, linkIndexB=b, physicsClientId=self.check_client)}

    def link_name(self, index):
        if index < 0:
            return self.base_name
        return p.getJointInfo(self.gui_body, index, physicsClientId=self.gui_client)[12].decode()

    def describe_collisions(self, pairs):
        return ", ".join(sorted(f"{self.link_name(a)} against {self.link_name(b)}" for a, b in pairs))


class Solution(NamedTuple):
    angles: list = None
    error: float = None
    reason: str = None
    detail: list = ()
    label: str = None


def solve(robot, target):
    target = np.asarray(target, dtype=float)

    distance = float(np.linalg.norm(target - robot.shoulder))
    if distance > robot.max_reach:
        return Solution(reason="that is beyond the arm's reach", label="out of reach", detail=[
            f"The point is {distance:.3f} m from the shoulder at "
            f"({robot.shoulder[0]:.3f}, {robot.shoulder[1]:.3f}, {robot.shoulder[2]:.3f}), and the links "
            f"can carry the tool at most {robot.max_reach:.3f} m from there."])

    rng = np.random.default_rng(0)
    seeds = [robot.midpoint] + [[rng.uniform(lo, hi) for lo, hi in zip(robot.lower, robot.upper)]
                                for _ in range(IK_RESTARTS)]

    short = limited = collided = 0
    closest_error = float("inf")
    limit_example = collision_example = None

    for seed in seeds:
        angles, error = _try_from(robot, target, seed)

        if error > POSITION_TOLERANCE:
            short += 1
            closest_error = min(closest_error, error)
            continue

        violations = robot.joint_limit_violations(angles)
        if violations:
            limited += 1
            limit_example = limit_example or violations[0]
            continue

        collisions = robot.self_collisions(angles)
        if collisions:
            collided += 1
            collision_example = collision_example or robot.describe_collisions(collisions)
            continue

        return Solution(angles=angles, error=error)

    attempts = len(seeds)
    detail = []
    if collided:
        detail.append(f"{collided} of {attempts} folded the arm into itself, "
                      f"one of them {collision_example}.")
    if limited:
        detail.append(f"{limited} of {attempts} needed a joint past its limit, "
                      f"one of them {limit_example}.")
    if short:
        detail.append(f"{short} of {attempts} could not reach it at all; none got the tool closer "
                      f"than {closest_error * 1000:.0f} mm.")
    return Solution(detail=detail, label="no pose found",
                    reason="the search did not find a legal pose")


def _try_from(robot, target, seed):
    robot.set_check_pose(seed)

    # Joint limits are checked in solve(), not passed here.
    angles = list(p.calculateInverseKinematics(robot.check_body, robot.end_effector, list(target),
                                               jointDamping=[IK_DAMPING] * len(robot.joints),
                                               maxNumIterations=200, residualThreshold=1e-6,
                                               physicsClientId=robot.check_client))[:len(robot.joints)]

    # Bullet can return an angle a whole turn off.
    angles = [(angle + np.pi) % (2 * np.pi) - np.pi for angle in angles]

    return angles, float(np.linalg.norm(robot.tool_position(angles) - target))


def quintic_path(start, goal, duration, waypoints):
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    travel = goal - start

    path, speeds = [], []
    for step in range(waypoints + 1):
        t = step / waypoints
        path.append(start + (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5) * travel)
        speeds.append((30 * t ** 2 - 60 * t ** 3 + 30 * t ** 4) / duration * travel)
    return path, speeds


def unsafe_between(robot, start, goal):
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    steps = max(1, int(np.ceil(ARM_RADIUS * float(np.abs(goal - start).sum()) / STEP_CLEARANCE)))

    for step in range(steps + 1):
        angles = start + (step / steps) * (goal - start)
        collisions = robot.self_collisions(angles)
        if collisions:
            return step / steps, robot.describe_collisions(collisions)
    return None
