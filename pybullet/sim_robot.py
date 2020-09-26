import time

import pybullet_data

import pybullet as p


class SimRobot:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        op3StartPos = [0, 0, 1]
        op3StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.planeId = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("../models/robotis_op3.urdf", op3StartPos, op3StartOrientation)
        self.numJoints = p.getNumJoints(self.robot)
        self.targetVel = 0
        self.maxForce = 100

    def _set_joint(self):
        for joint in range(self.numJoints):
            print(p.getJointInfo(self.robot, joint)[1])
            p.setJointMotorControl(self.robot, joint, p.POSITION_CONTROL, self.targetVel, self.maxForce)

    def run(self):
        try:
            while True:
                p.stepSimulation()
                self._set_joint()
                time.sleep(1. / 240.)
        finally:
            OP3Pos, OP3Orn = p.getBasePositionAndOrientation(self.robot)
            print(OP3Pos, OP3Orn)
            p.disconnect()


if __name__ == '__main__':
    op3 = SimRobot()
    op3.run()
