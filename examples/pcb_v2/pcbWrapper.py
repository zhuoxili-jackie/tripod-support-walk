import os
import numpy as np  # Linear Algebra

import pinocchio as pin  # Pinocchio library


from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.visualize import MeshcatVisualizer

class pcb:
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(dir_path, "pcb_v2/urdf/pcb_v2.urdf")
        model_path = os.path.join(dir_path, "pcb_v2")

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, package_dirs=[model_path]
        )

        self.data = self.model.createData()

        self.left_foot_frames = ["RL_foot_link"]
        self.right_foot_frames = ["RR_foot_link"]
        self.left_gripper_frames = ["FL_foot_link"]
        self.right_gripper_frames = ["FR_foot_link"]

    def fk_all(self, q, v=None):
        if v is not None:
            pin.forwardKinematics(
                self.model, self.data, q, v
            )  # FK and Forward Velocities
        else:
            pin.forwardKinematics(self.model, self.data, q)  # FK
        pin.updateFramePlacements(self.model, self.data)  # Update frames

    def go_neutral(self):
        q = pin.neutral(self.model)
        # q[2] = 0.45
        # q[8] = 0.81
        # q[9] = -1.535
        # q[11] = 0.81
        # q[12] = -1.535
        # q[14] = -0.81
        # q[15] = 1.535
        # q[17] = -0.81
        # q[18] = 1.535
        q[0] = 0.0
        q[1] = 0.0
        q[2] = 0.72963
        q[3] = 0
        q[4] = -0.5230950117111206
        q[5] = 0
        q[6] = 0.8522081971168518
        q[7]=0
        q[8] = -0.81
        q[9] = 0.872
        q[10]=0
        q[11] = -0.81
        q[12] = 0.872
        q[13]=0
        q[14] = 0.65
        q[15] = 1.0
        q[16]=0
        q[17] = 0.65
        q[18] = 1.0
        self.fk_all(q)
        return q


if __name__ == "__main__":
    pcb = pcb()
    # talos.load_visualizer()
    q = pcb.go_neutral()
    # talos.display(q)

    pcb.fk_all(q)

    for i, frame in enumerate(pcb.model.frames):
        print(
            f"Frame {i}: Name = {frame.name}, Type = {frame.type}, Parrent = {frame.parentJoint} "
        )

    print(pcb.model)

    # for i in range(pcb.model.fr):
    # print(i, pcb.data.joints[i].M)
