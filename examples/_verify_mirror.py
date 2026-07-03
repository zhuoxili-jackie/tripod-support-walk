#!/usr/bin/env python3
"""Verify a LEFT (+Y) sideways gait is the exact Y-mirror of its RIGHT (-Y) twin.

A perfect reflection about the sagittal (XZ) plane requires, at every frame:
  base:   x_L = x_R,  y_L = -y_R,  z_L = z_R
  orient: roll_L = -roll_R,  pitch_L = pitch_R,  yaw_L = -yaw_R
  feet (with L<->R swap): pos_L[FR] = mirrorY(pos_R[FL]) and FL<->FR, RL<->RR,
    where mirrorY negates the y component.
Prints the MAX abs deviation of each channel over the whole trajectory.
Sub-mm / sub-mrad => a true mirror to numeric precision (NOT bit-exact, since
each direction is an independent FDDP solve and the URDF is not perfectly
Y-symmetric -- see CONTINUATION_PROMPT §六 v1.5).

Usage (inside examples/):
    python _verify_mirror.py                          # sweep the 4 committed speeds
    python _verify_mirror.py <right.csv> <left.csv>   # one explicit pair
"""
import csv
import os
import sys

import numpy as np
import pinocchio
from pinocchio.robot_wrapper import RobotWrapper

dir_path = os.path.dirname(os.path.realpath(__file__))
robot = RobotWrapper.BuildFromURDF(
    os.path.join(dir_path, "pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf"),
    os.path.join(dir_path, "pcb_v2", "pcb_v2"))
m, d = robot.model, robot.data
feet = ["FL_foot_link", "FR_foot_link", "RL_foot_link", "RR_foot_link"]
fid = {f: m.getFrameId(f) for f in feet}
swap = {"FL_foot_link": "FR_foot_link", "FR_foot_link": "FL_foot_link",
        "RL_foot_link": "RR_foot_link", "RR_foot_link": "RL_foot_link"}


def load(path):
    rows = list(csv.reader(open(path)))[1:]
    return np.array([[float(x) for x in r] for r in rows])


def euler(quat_xyzw):
    x, y, z, w = quat_xyzw.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def feet_pos(A):
    out = {f: np.zeros((len(A), 3)) for f in feet}
    for i in range(len(A)):
        q = np.zeros(m.nq)
        q[0:3] = A[i, 0:3]; q[3:7] = A[i, 3:7]; q[7:] = A[i, 7:7 + (m.nq - 7)]
        pinocchio.forwardKinematics(m, d, q)
        pinocchio.updateFramePlacements(m, d)
        for f in feet:
            out[f][i] = d.oMf[fid[f]].translation
    return out


def compare(right_csv, left_csv, tag):
    R, L = load(right_csv), load(left_csv)
    n = min(len(R), len(L))
    R, L = R[:n], L[:n]
    dx = np.max(np.abs(L[:, 0] - R[:, 0])) * 1e3
    dy = np.max(np.abs(L[:, 1] + R[:, 1])) * 1e3          # y_L = -y_R
    dz = np.max(np.abs(L[:, 2] - R[:, 2])) * 1e3
    rR, pR, yR = euler(R[:, 3:7]); rL, pL, yL = euler(L[:, 3:7])
    droll = np.max(np.abs(rL + rR)) * 1e3                 # roll_L = -roll_R
    dpitch = np.max(np.abs(pL - pR)) * 1e3                # pitch_L = pitch_R
    dyaw = np.max(np.abs(yL + yR)) * 1e3                  # yaw_L = -yaw_R
    fR, fL = feet_pos(R), feet_pos(L)
    dfeet = 0.0
    for f in feet:
        mir = fR[f].copy(); mir[:, 1] *= -1
        dfeet = max(dfeet, np.max(np.linalg.norm(fL[swap[f]] - mir, axis=1)))
    print(f"{tag:8s}{dx:>9.3f}{dy:>9.3f}{dz:>9.3f}{droll:>9.3f}{dpitch:>9.3f}"
          f"{dyaw:>9.3f}{dfeet * 1e3:>12.3f}")


def main():
    print(f"{'pair':8s}{'base dx':>9s}{'base dy':>9s}{'base dz':>9s}"
          f"{'roll':>9s}{'pitch':>9s}{'yaw':>9s}{'feet(swap)':>12s}")
    print(f"{'':8s}{'(mm)':>9s}{'(mm)':>9s}{'(mm)':>9s}{'(mrad)':>9s}"
          f"{'(mrad)':>9s}{'(mrad)':>9s}{'(mm)':>12s}")
    if len(sys.argv) == 3:
        compare(sys.argv[1], sys.argv[2], "pair")
    else:
        for s in ["0.05", "0.10", "0.15", "0.20"]:
            r = os.path.join(dir_path, f"trajectory_walking_sideways_sc_v{s}.csv")
            l = os.path.join(dir_path, f"trajectory_walking_sideways_sc_left_v{s}.csv")
            if os.path.exists(r) and os.path.exists(l):
                compare(r, l, f"v{s}")


if __name__ == "__main__":
    main()
