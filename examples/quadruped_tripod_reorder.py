#!/usr/bin/env python3
"""Step 2 (§0.6-2): tripod gait with the swing order FR -> RL -> RR -> FL.

Same driver as quadruped_walking_fwddyn.py (stock createWalkingProblem, NO
library edit; symmetric cycle-0 ramp; stride x cadence speed scaling; right/-Y
with an exact left/+Y mirror) EXCEPT the swing order is FR -> RL -> RR -> FL
instead of the baseline FL -> RL -> FR -> RR.

createWalkingProblem always swings rh -> rf -> lh -> lf, so an arbitrary order
[A,B,C,D] is pure foot-name re-binding rh=A, rf=B, lh=C, lf=D -- no library edit.
    RIGHT (-Y): FR->RL->RR->FL  ->  rh=FR, rf=RL, lh=RR, lf=FL
    LEFT (+Y): the exact L<->R mirror (swap FL<->FR, RL<->RR) -> FL->RR->RL->FR,
               rh=FL, rf=RR, lh=RL, lf=FR, direction +Y (matches the baseline's
               mirror recipe so the cold-start conditioning mirrors too).

Purpose: reordering is the cheap first try. _verify_fall.py + _verify_tripod_com.py
confirm it does NOT fix the tip (any order keeps comRef = four-foot average x~0.06,
so hind swings still leave the CoM behind the x~0.30 support triangle and the
diagonal front foot still pulls). It only halves the severity / smooths the
"hind-leg tuck-in" transient. The real fix is step 3 (quadruped_tripod_com.py).

Run (examples/, croco310, build_conda on PYTHONPATH):
    python quadruped_tripod_reorder.py                     # right 0.05
    python quadruped_tripod_reorder.py --speed 0.20        # right 0.20
    python quadruped_tripod_reorder.py --direction left    # left  0.05 (mirror)
"""
import argparse
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

parser = argparse.ArgumentParser()
parser.add_argument("--speed", type=float, default=0.05)
parser.add_argument("--cadence-share", type=float, default=0.5,
                    help="alpha in [0,1]: cadenceFactor=R**alpha, strideFactor=R**(1-alpha), "
                         "R=speed/0.05 (same as the baseline driver)")
parser.add_argument("--direction", choices=["right", "left"], default="right")
parser.add_argument("--out", type=str, default=None)
args = parser.parse_args()

dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(dir_path, "pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf")
mesh_dir = os.path.join(dir_path, "pcb_v2", "pcb_v2")

robot = pcb()
robot_pcb = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
robot_pcb.model.referenceConfigurations["standing"] = robot.go_neutral()
q0 = robot_pcb.model.referenceConfigurations["standing"].copy()
x0 = np.concatenate([q0, pinocchio.utils.zero(robot_pcb.model.nv)])

# swing order FR->RL->RR->FL (right); its exact L<->R mirror FL->RR->RL->FR (left)
if args.direction == "right":
    lfFoot, rfFoot, lhFoot, rhFoot = (
        "FL_foot_link", "RL_foot_link", "RR_foot_link", "FR_foot_link",
    )
else:  # left = swap FL<->FR, RL<->RR of the right binding
    lfFoot, rfFoot, lhFoot, rhFoot = (
        "FR_foot_link", "RR_foot_link", "RL_foot_link", "FL_foot_link",
    )
gait = SimpleQuadrupedalGaitProblem(robot_pcb.model, lfFoot, rfFoot, lhFoot, rhFoot)
gait.firstStep = False  # symmetric cycle-0 ramp instead (jitter fix, §0.5-E)

# gait params (identical scaling to quadruped_walking_fwddyn.py)
N_CYCLES = 8
FIRST_CYCLE_STRIDE_FRAC = 0.5
timeStep = 0.01
BASE_SPEED, BASE_STEP_KNOTS, BASE_SUPPORT_KNOTS = 0.05, 35, 10
MIN_STEP_KNOTS, MIN_SUPPORT_KNOTS = 8, 3
stepHeight = 0.10
dir_sign = -1.0 if args.direction == "right" else 1.0
direction = (0.0, dir_sign)

R = args.speed / BASE_SPEED
cadenceFactor = R ** args.cadence_share
stepKnots = max(MIN_STEP_KNOTS, int(BASE_STEP_KNOTS / cadenceFactor + 0.5))
supportKnots = max(MIN_SUPPORT_KNOTS, int(BASE_SUPPORT_KNOTS / cadenceFactor + 0.5))
T_step_cycle = (2 * supportKnots + 4 * stepKnots) * timeStep
stepLength = args.speed * T_step_cycle

dir_tag = "" if args.direction == "right" else "left_"
out = args.out or f"trajectory_tripod_reorder_{dir_tag}v{args.speed:.2f}.csv"
print(f"speed={args.speed} dir={args.direction} R={R:.3f} stepKnots={stepKnots} "
      f"supportKnots={supportKnots} stepLength={stepLength:.4f} -> {out}")

solvers = []
for i in range(N_CYCLES):
    cyc = FIRST_CYCLE_STRIDE_FRAC * stepLength if i == 0 else stepLength
    problem = gait.createWalkingProblem(x0, cyc, stepHeight, timeStep, stepKnots,
                                        supportKnots, direction=direction)
    solver = crocoddyl.SolverFDDP(problem)
    xs = [x0] * (solver.problem.T + 1)
    us = solver.problem.quasiStatic([x0] * solver.problem.T)
    solver.solve(xs, us, 200, False)
    print(f"  cycle {i}: iters={solver.iter} stop={solver.stop:.2e} feas={solver.isFeasible}")
    solvers.append(solver)
    x0 = solver.xs[-1]

joint_names = [robot_pcb.model.names[i] for i in range(2, robot_pcb.model.njoints)]
fieldnames = (["root_pos_x", "root_pos_y", "root_pos_z",
               "root_rot_x", "root_rot_y", "root_rot_z", "root_rot_w"]
              + joint_names
              + ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"])
bfid = robot_pcb.model.getFrameId("Base_link")
n = 0
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    for solver in solvers:
        for x in solver.xs:
            q = x[: robot_pcb.model.nq]
            pinocchio.forwardKinematics(robot_pcb.model, robot_pcb.data, q)
            pinocchio.updateFramePlacements(robot_pcb.model, robot_pcb.data)
            bp = robot_pcb.data.oMf[bfid]
            quat = pinocchio.SE3ToXYZQUAT(bp)[3:7]
            row = {"root_pos_x": bp.translation[0], "root_pos_y": bp.translation[1],
                   "root_pos_z": bp.translation[2], "root_rot_x": quat[0],
                   "root_rot_y": quat[1], "root_rot_z": quat[2], "root_rot_w": quat[3]}
            for j, val in enumerate(q[7:]):
                row[joint_names[j]] = val
            for fj in ("FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"):
                row[fj] = 0.0
            w.writerow(row)
            n += 1
print(f"wrote {out}: {n} rows x {len(fieldnames)} cols")
