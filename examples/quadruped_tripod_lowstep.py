#!/usr/bin/env python3
"""Tripod gait: 0706 swing order (FR->RL->RR->FL) + 0703 CoM approach (NO lean) +
low ~2 cm foot lift + fore-aft trunk anti-sway (2026-07-07, posture-first).

Background: the 0706 driver (quadruped_tripod_com.py) actively leaned the CoM
forward into each hind support triangle, which pitched the trunk / raised the
butt (抬屁股). Per the user we dropped that and went back to the 0703 stock
approach (comRef = four-foot average, no active CoM shift) keeping only the 0706
swing order and lowering the foot lift to ~2 cm. That fixed the butt-raise, but
because the vertical lift is now tiny the residual FORE-AFT trunk sway became
visible: when the two LEFT legs (FL, RL) swing during a rightward walk the trunk
rocks ~2 cm fore-aft (and mirror legs for a leftward walk).

Why it rocks: the stock comTrack pins the CoM in x at weight 1e6, so as a leg
swings the trunk must recoil fore-aft to keep the CoM x fixed (measured: the base
x oscillates ~5 cm peak-to-peak per cycle, recoiling hardest when the left legs
finish their swing). The user does not care where the CoM sits, only that the
trunk stays steady -- so the fix RELAXES the CoM x-tracking (``--com-x-weight``,
default 0.1 vs the stock 1.0) via the library's optional per-axis ``comWeights``.
The CoM is then free to wiggle fore-aft harmlessly while the trunk stops
recoiling: base-x sway drops from ~4.9 cm to ~1.8 cm whole-trajectory (left-leg
surge ~2.0 cm -> ~1.1 cm), CoM x-drift actually shrinks, and the -Y walk, the
2 cm lift and the flat posture are all unchanged. y/z CoM tracking stays pinned
(y drives the walk, z holds the height).

2026-07-07 posture-first update: the user now prefers as little fore-aft trunk
tilt as possible and accepts visible hind knock-knee. Therefore the default hip
regularization is back to stock (``--hip-reg 50`` and ``--hipvel-reg 1``). This
keeps pitch/body sway smallest in the low-step setup, at the cost of larger hind
hip adduction (inner-knee look).

Everything else matches quadruped_tripod_reorder.py: stock double-support +
per-foot swing phases, symmetric cycle-0 ramp, stride x cadence speed scaling,
swing order FR->RL->RR->FL with an exact left/+Y mirror.

createFootstepModels swings the named foot with three feet in support; the order
is chosen by which real foot each phase swings:
    RIGHT (-Y): FR -> RL -> RR -> FL
    LEFT (+Y): the exact L<->R mirror (swap FL<->FR, RL<->RR) -> FL -> RR -> RL -> FR.

Library note: this uses the optional ``comWeights`` argument added to
createModel/createFootstepModels (default None = stock, so trot/walking and every
other gait are byte-identical). The build_conda copy must be refreshed after any
library edit (cp bindings/.../quadruped.py to build_conda/.../quadruped.py).

Run (examples/, croco310, build_conda on PYTHONPATH):
    python quadruped_tripod_lowstep.py                      # right 0.05, 2cm lift
    python quadruped_tripod_lowstep.py --speed 0.20         # right 0.20
    python quadruped_tripod_lowstep.py --direction left     # left 0.05 (mirror)
    python quadruped_tripod_lowstep.py --com-x-weight 1.0   # stock CoM pin (sway back)
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

FOOT = {"FL": "FL_foot_link", "FR": "FR_foot_link",
        "RL": "RL_foot_link", "RR": "RR_foot_link"}

parser = argparse.ArgumentParser()
parser.add_argument("--speed", type=float, default=0.05)
parser.add_argument("--cadence-share", type=float, default=0.5,
                    help="alpha in [0,1]: cadenceFactor=R**alpha, strideFactor=R**(1-alpha), "
                         "R=speed/0.05 (same as the baseline driver)")
parser.add_argument("--front-stepheight", type=float, default=0.016,
                    help="front-foot swing clearance (m); ~2cm actual peak lift. Front "
                         "tracks a touch above target under the relaxed CoM, so 0.016->~2cm.")
parser.add_argument("--hind-stepheight", type=float, default=0.026,
                    help="hind-foot swing clearance (m); ~2cm actual peak lift. Hind "
                         "tracks a touch below target, so 0.026->~2cm.")
parser.add_argument("--com-x-weight", type=float, default=0.1,
                    help="fore-aft (x) CoM-tracking weight relative to y,z (stock 1.0). "
                         "Lower it (default 0.1) to let the CoM float fore-aft so the "
                         "trunk stops recoiling / swaying when the left legs swing. "
                         "1.0 = stock pin (sway returns); y,z stay pinned.")
parser.add_argument("--hip-reg", type=float, default=50.0,
                    help="hip (lateral/ab-adduction) position weight. 50 is stock and "
                         "prioritizes minimal fore-aft trunk tilt/sway while accepting "
                         "visible hind knock-knee (内八). Raising it trims 内八 but "
                         "re-introduces body pitch/sway.")
parser.add_argument("--hipvel-reg", type=float, default=1.0,
                    help="hip joint-VELOCITY weight. 1 is stock and gives the smallest "
                         "posture motion for the accepted-内八 lowstep version. Raise "
                         "toward 25 only when using stiffer --hip-reg to damp tremble.")
parser.add_argument("--joint-reg", type=float, default=50.0,
                    help="thigh+calf position weight (stock 50); keep at 50.")
parser.add_argument("--order", type=str, default="FR,RL,RR,FL",
                    help="right-walk swing order; left mirrors L<->R automatically")
parser.add_argument("--direction", choices=["right", "left"], default="right")
parser.add_argument("--out", type=str, default=None)
args = parser.parse_args()

dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(dir_path, "pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf")
mesh_dir = os.path.join(dir_path, "pcb_v2", "pcb_v2")

robot = pcb()
robot_pcb = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
robot_pcb.model.referenceConfigurations["standing"] = robot.go_neutral()
model = robot_pcb.model
q0 = model.referenceConfigurations["standing"].copy()
x0 = np.concatenate([q0, pinocchio.utils.zero(model.nv)])

# swing order FR->RL->RR->FL (right); its exact L<->R mirror FL->RR->RL->FR (left)
order = [FOOT[k.strip()] for k in args.order.split(",")]
if args.direction == "left":
    LR = {"FL_foot_link": "FR_foot_link", "FR_foot_link": "FL_foot_link",
          "RL_foot_link": "RR_foot_link", "RR_foot_link": "RL_foot_link"}
    order = [LR[n] for n in order]
ALL = [FOOT[k] for k in ("FL", "FR", "RL", "RR")]

gait = SimpleQuadrupedalGaitProblem(model, FOOT["FL"], FOOT["FR"], FOOT["RL"], FOOT["RR"])

# gait params (identical scaling to quadruped_walking_fwddyn.py / _reorder.py)
N_CYCLES = 8
FIRST_CYCLE_STRIDE_FRAC = 0.5
timeStep = 0.01
BASE_SPEED, BASE_STEP_KNOTS, BASE_SUPPORT_KNOTS = 0.05, 35, 10
MIN_STEP_KNOTS, MIN_SUPPORT_KNOTS = 8, 3
FRONT = {FOOT["FL"], FOOT["FR"]}
dir_sign = -1.0 if args.direction == "right" else 1.0
direction = (0.0, dir_sign)
# relax fore-aft CoM tracking so the trunk needn't recoil (y,z stay pinned)
comWeights = np.array([args.com_x_weight, 1.0, 1.0])
# state regularisation: raise the hip-POSITION weight to trim the hind 内八 (knock-
# knee), and the hip-VELOCITY weight to damp the tremble a stiffer hip cost induces.
# base position [0]*3 (anti-sway comes from comWeights, not base reg); base ori 500
# (stock, keeps the trunk flat -- no tilt); thigh/calf 50 (stock, no crouch); base
# vel [10]*6; only the 4 hip velocities raised so thigh/calf stay free (foot lift).
# Layout: [0]*3 + [500]*3 + [hip,thigh,calf]x4 + [10]*6 + [hipvel,1,1]x4.
_jp, _jv = [], []
for _ in range(4):                      # FL, FR, RL, RR
    _jp += [args.hip_reg, args.joint_reg, args.joint_reg]
    _jv += [args.hipvel_reg, 1.0, 1.0]
stateWeights = np.array([0.0] * 3 + [500.0] * 3 + _jp + [10.0] * 6 + _jv)

R = args.speed / BASE_SPEED
cadenceFactor = R ** args.cadence_share
stepKnots = max(MIN_STEP_KNOTS, int(BASE_STEP_KNOTS / cadenceFactor + 0.5))
supportKnots = max(MIN_SUPPORT_KNOTS, int(BASE_SUPPORT_KNOTS / cadenceFactor + 0.5))
T_step_cycle = (2 * supportKnots + 4 * stepKnots) * timeStep
stepLength = args.speed * T_step_cycle


def build_walking(x0, stepLength):
    """Replicate createWalkingProblem EXACTLY (double-support only between the two
    swing pairs: ds + s0 + s1 + ds + s2 + s3 + [ds0]) but forward comWeights so the
    fore-aft CoM tracking can be relaxed. comRef = four-foot average (stock, no
    lean); createFootstepModels mutates comRef/feetPos in place so they accumulate
    across the sequential swings, matching the stock factory."""
    q = x0[: model.nq]
    pinocchio.forwardKinematics(model, gait.rdata, q)
    pinocchio.updateFramePlacements(model, gait.rdata)
    fid = {n: model.getFrameId(n) for n in ALL}
    feetPos = {n: gait.rdata.oMf[fid[n]].translation.copy() for n in ALL}
    comRef = sum(feetPos[n] for n in ALL) / 4.0
    comRef[2] = pinocchio.centerOfMass(model, gait.rdata, q)[2].item()

    ds = [gait.createModel(timeStep=timeStep, footContacts=ALL, stateWeights=stateWeights)
          for _ in range(supportKnots)]

    def fs(swing):
        support = [n for n in ALL if n != swing]
        sh = args.front_stepheight if swing in FRONT else args.hind_stepheight
        return gait.createFootstepModels(
            comRef, [feetPos[swing]], stepLength, sh, timeStep, stepKnots,
            support, [swing], direction=direction, comWeights=comWeights,
            stateWeights=stateWeights,
        )

    s0, s1, s2, s3 = order
    loco = ds + fs(s0) + fs(s1) + ds + fs(s2) + fs(s3) + [ds[0]]
    return crocoddyl.ShootingProblem(x0, loco[:-1], loco[-1])


dir_tag = "" if args.direction == "right" else "left_"
out = args.out or f"trajectory_tripod_lowstep_{dir_tag}v{args.speed:.2f}.csv"
print(f"speed={args.speed} dir={args.direction} order={'->'.join(n[:2] for n in order)} "
      f"R={R:.3f} stepKnots={stepKnots} supportKnots={supportKnots} "
      f"stepLength={stepLength:.4f} front/hind-sh={args.front_stepheight:.3f}/{args.hind_stepheight:.3f} "
      f"com_x_w={args.com_x_weight} hip_reg={args.hip_reg} -> {out}")

solvers = []
for i in range(N_CYCLES):
    cyc = FIRST_CYCLE_STRIDE_FRAC * stepLength if i == 0 else stepLength
    problem = build_walking(x0, cyc)
    solver = crocoddyl.SolverFDDP(problem)
    xs = [x0] * (solver.problem.T + 1)
    us = solver.problem.quasiStatic([x0] * solver.problem.T)
    solver.solve(xs, us, 200, False)
    print(f"  cycle {i}: T={solver.problem.T} iters={solver.iter} "
          f"stop={solver.stop:.2e} feas={solver.isFeasible}")
    solvers.append(solver)
    x0 = solver.xs[-1]

joint_names = [model.names[i] for i in range(2, model.njoints)]
fieldnames = (["root_pos_x", "root_pos_y", "root_pos_z",
               "root_rot_x", "root_rot_y", "root_rot_z", "root_rot_w"]
              + joint_names
              + ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"])
bfid = model.getFrameId("Base_link")
n = 0
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    for solver in solvers:
        for x in solver.xs:
            q = x[: model.nq]
            pinocchio.forwardKinematics(model, robot_pcb.data, q)
            pinocchio.updateFramePlacements(model, robot_pcb.data)
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
