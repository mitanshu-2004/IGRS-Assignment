# Measures the metre that puma.ARM_RADIUS assumes.

import numpy as np
import pybullet as p

from puma import ARM_RADIUS, Puma560


def main():
    robot = Puma560(p.connect(p.DIRECT), p.connect(p.DIRECT))
    meshes = {link: np.array(p.getMeshData(robot.check_body, link,
                                           physicsClientId=robot.check_client)[1])
              for link in robot.joints}
    rng = np.random.default_rng(0)
    worst = 0.0

    for _ in range(400):
        robot.set_check_pose([rng.uniform(lo, hi) for lo, hi in zip(robot.lower, robot.upper)])
        for joint in robot.joints:
            frame = p.getLinkState(robot.check_body, joint, computeForwardKinematics=True,
                                   physicsClientId=robot.check_client)
            origin = np.array(frame[4])
            axis = np.array(p.getMatrixFromQuaternion(frame[5])).reshape(3, 3) @ np.array(
                p.getJointInfo(robot.check_body, joint, physicsClientId=robot.check_client)[13])

            for carried in range(joint, len(robot.joints)):
                pose = p.getLinkState(robot.check_body, carried, computeForwardKinematics=True,
                                      physicsClientId=robot.check_client)
                turn = np.array(p.getMatrixFromQuaternion(pose[5])).reshape(3, 3)
                points = meshes[carried] @ turn.T + np.array(pose[4]) - origin
                worst = max(worst, float(np.linalg.norm(
                    points - np.outer(points @ axis, axis), axis=1).max()))

    print(f"Furthest a carried point sat from its joint's axis over the sampled poses: {worst:.3f} m")
    print(f"The path spacing assumes no more than {ARM_RADIUS:.1f} m")


if __name__ == "__main__":
    main()
