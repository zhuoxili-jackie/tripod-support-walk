#!/usr/bin/env python3
"""Single-leg-swing (tripod) gait via the STOCK crocoddyl walking gait.

Uses ``SimpleQuadrupedalGaitProblem.createWalkingProblem`` exactly as shipped --
NO edits to the crocoddyl library. That factory already swings one foot at a
time with three feet in support (rh -> rf -> lh -> lf per cycle). Here it walks
sideways (-Y), matching the reference ``trajectory_single_leg_acc_f005.csv``.

Sibling of ``quadruped_gaits_fwddyn.py`` (the side-trot driver); neither it nor
the library is modified. Same 23-column / 100 Hz CSV contract (RUN_PIPELINE §4).

Run (inside examples/ so ``import pcb_v2`` works):
    conda activate croco310
    export PYTHONPATH=<repo>/build_conda/bindings/python:<repo>/examples
    cd <repo>/examples
    python quadruped_walking_fwddyn.py
"""
import csv
import os
import signal

import numpy as np
import pinocchio

import crocoddyl
from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem
from pcb_v2.pcbWrapper import pcb

signal.signal(signal.SIGINT, signal.SIG_DFL)
from pinocchio.robot_wrapper import RobotWrapper  # noqa: E402

dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(dir_path, "pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf")
mesh_dir = os.path.join(dir_path, "pcb_v2", "pcb_v2")

robot = pcb()
robot_pcb = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
robot_pcb.model.referenceConfigurations["standing"] = robot.go_neutral()

q0 = robot_pcb.model.referenceConfigurations["standing"].copy()
v0 = pinocchio.utils.zero(robot_pcb.model.nv)
x0 = np.concatenate([q0, v0])

# createWalkingProblem always swings in the order rh -> rf -> lh -> lf, so the
# swing order is chosen purely by which real foot each role-name is bound to
# (a driver-level choice, no library edit). Binding rh=FL, rf=RL, lh=FR, lf=RR
# reproduces the reference CSV's FL -> RL -> FR -> RR order. Use the natural
# mapping (lf=FL, rf=FR, lh=RL, rh=RR) instead for the stock RR,FR,RL,FL order.
lfFoot, rfFoot, lhFoot, rhFoot = (
    "RR_foot_link", "RL_foot_link", "FR_foot_link", "FL_foot_link",
)
gait = SimpleQuadrupedalGaitProblem(robot_pcb.model, lfFoot, rfFoot, lhFoot, rhFoot)

# --- gait parameters (8 cycles x 165 rows = 1320, like the reference CSV) ----
N_CYCLES = 8
timeStep = 0.01          # 100 Hz
stepKnots = 35           # swing-phase nodes
supportKnots = 10        # double-support nodes
stepLength = 0.08        # per-cycle sideways displacement
stepHeight = 0.10        # swing clearance
direction = (0.0, -1.0)  # walk -Y (sideways), matching the reference

output_path = "trajectory_walking_sideways.csv"

solvers = []
for i in range(N_CYCLES):
    problem = gait.createWalkingProblem(
        x0, stepLength, stepHeight, timeStep, stepKnots, supportKnots,
        direction=direction,
    )
    solver = crocoddyl.SolverFDDP(problem)
    solver.setCallbacks([crocoddyl.CallbackVerbose()])
    xs = [x0] * (solver.problem.T + 1)
    us = solver.problem.quasiStatic([x0] * solver.problem.T)
    print(f"\n*** SOLVE cycle {i} (FDDP, T={solver.problem.T}) ***")
    solver.solve(xs, us, 200, False)
    print(f"cycle {i}: iters={solver.iter} stop={solver.stop:.3e} "
          f"cost={solver.cost:.4f} isFeasible={solver.isFeasible}")
    solvers.append(solver)
    x0 = solver.xs[-1]

# --- export 23-column CSV ----------------------------------------------------
joint_names = [robot_pcb.model.names[i] for i in range(2, robot_pcb.model.njoints)]
fieldnames = (
    ["root_pos_x", "root_pos_y", "root_pos_z",
     "root_rot_x", "root_rot_y", "root_rot_z", "root_rot_w"]
    + joint_names
    + ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]
)
base_frame_id = robot_pcb.model.getFrameId("Base_link")
n_rows = 0
with open(output_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    for solver in solvers:
        for x in solver.xs:
            q = x[: robot_pcb.model.nq]
            pinocchio.forwardKinematics(robot_pcb.model, robot_pcb.data, q)
            pinocchio.updateFramePlacements(robot_pcb.model, robot_pcb.data)
            bp = robot_pcb.data.oMf[base_frame_id]
            pos = bp.translation.copy()
            quat = pinocchio.SE3ToXYZQUAT(bp)[3:7]
            row = {"root_pos_x": pos[0], "root_pos_y": pos[1], "root_pos_z": pos[2],
                   "root_rot_x": quat[0], "root_rot_y": quat[1],
                   "root_rot_z": quat[2], "root_rot_w": quat[3]}
            for j, val in enumerate(q[7:]):
                row[joint_names[j]] = val
            for fj in ("FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"):
                row[fj] = 0.0
            w.writerow(row)
            n_rows += 1

print("\nSaving to:", os.path.abspath(output_path))
print(f"Done. {len(solvers)} cycles, {n_rows} rows, {len(fieldnames)} columns.")
with open(output_path) as f:
    lines = f.read().splitlines()
print("#columns =", len(lines[0].split(",")), " #data rows =", len(lines) - 1)
print("row[0]:", lines[1])
print("row[1]:", lines[2])
