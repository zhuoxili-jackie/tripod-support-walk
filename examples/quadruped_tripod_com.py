#!/usr/bin/env python3
"""Step 3 (§0.6-3, revised 2026-07-03 per user): tripod gait that leans the CoM
toward each support triangle but caps the trunk tilt at ~5 deg, accepting that
hind swings may not fully clear the tip (a later RL policy rewards the rest).

Earlier this driver drove the CoM all the way into the hind triangle -> FALL 0
but at the cost of a ~18 deg trunk pitch and, when the joint regulariser was
relaxed, a 59 deg hip splay (knock-knee / 内八). The user's revised call: keep
the trunk NEARLY LEVEL (<= ~5 deg), keep the legs natural (no knock-knee), lift
the feet higher, and let the hind swings tip a little (RL training compensates).

How the tilt cap and natural legs are enforced: the library createModel/
createFootstepModels now take an optional ``stateWeights`` vector (default None =
stock, so trot/walking are unchanged). This driver passes a vector that
  * lowers base-orientation 500 -> ``--base-ori-reg`` (250 gives ~5 deg tilt), and
  * keeps hip and thigh/calf at the stock 50 (no splay, no deep crouch).
The CoM is still nudged toward the triangle by the per-swing comTask + pre-shift,
but the ~5 deg trunk cap means it falls short on hind swings -> they tip a bit.

Feet lift: front feet swing to ``--front-stepheight`` (0.15 m), hind to
``--hind-stepheight`` (0.10 m).

Run (examples/, croco310, build_conda on PYTHONPATH; library already cp'd there):
    python quadruped_tripod_com.py                 # tilt~5deg, front 15cm / hind 10cm
    python _verify_fall.py trajectory_tripod_com_*.csv
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
FRONT = {"FL_foot_link", "FR_foot_link"}


def closest_point_in_triangle(p, a, b, c):
    """Closest point to p inside triangle abc (2D). p if inside, else nearest
    boundary point (Ericson, Real-Time Collision Detection)."""
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = ab @ ap, ac @ ap
    if d1 <= 0 and d2 <= 0:
        return a.copy()
    bp = p - b
    d3, d4 = ab @ bp, ac @ bp
    if d3 >= 0 and d4 <= d3:
        return b.copy()
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return a + (d1 / (d1 - d3)) * ab
    cp = p - c
    d5, d6 = ab @ cp, ac @ cp
    if d6 >= 0 and d5 <= d6:
        return c.copy()
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return a + (d2 / (d2 - d6)) * ac
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b)
    denom = 1.0 / (va + vb + vc)
    return a + ab * (vb * denom) + ac * (vc * denom)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type=float, default=0.65,
                    help="shrink the support triangle toward its centroid; the CoM "
                         "target is this far off every edge (only partly reached "
                         "under the tilt cap)")
    ap.add_argument("--stride", type=float, default=0.06,
                    help="per-swing foot displacement at 0.05 m/s (scaled by --speed)")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="avg sideways speed (m/s); scales stride x cadence like the baseline driver")
    ap.add_argument("--cadence-share", type=float, default=0.5,
                    help="alpha: cadence=R**alpha, stride=R**(1-alpha), R=speed/0.05")
    ap.add_argument("--front-stepheight", type=float, default=0.12,
                    help="front-foot swing clearance param (~15cm actual peak; front feet "
                         "on the box arc higher than the param)")
    ap.add_argument("--hind-stepheight", type=float, default=0.115,
                    help="hind-foot swing clearance param (~10cm actual peak)")
    ap.add_argument("--stepknots", type=int, default=35)
    ap.add_argument("--preshift", type=int, default=20, help="4-foot CoM-shift nodes before each swing")
    ap.add_argument("--hold", type=int, default=15, help="4-foot settle nodes after each swing")
    ap.add_argument("--cycles", type=int, default=8)
    ap.add_argument("--maxiter", type=int, default=400)
    ap.add_argument("--order", type=str, default="FR,RL,RR,FL", help="swing order (FL/FR/RL/RR)")
    ap.add_argument("--direction", choices=["right", "left"], default="right")
    ap.add_argument("--base-ori-reg", type=float, default=250.0,
                    help="base-orientation weight (stock 500). 250 caps trunk tilt at "
                         "~5 deg; lower = more tilt / more CoM reach, higher = flatter")
    ap.add_argument("--hip-reg", type=float, default=50.0,
                    help="hip (lateral) weight (stock 50). Keep at 50 -> no knock-knee/内八")
    ap.add_argument("--joint-reg", type=float, default=50.0,
                    help="thigh+calf weight (stock 50). Keep at 50 -> no deep crouch")
    ap.add_argument("--constraint", action="store_true", help="hard friction cones (SolverIntro)")
    ap.add_argument("--out", type=str, default=None)
    args = ap.parse_args()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    urdf = os.path.join(dir_path, "pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf")
    mesh = os.path.join(dir_path, "pcb_v2", "pcb_v2")
    robot = pcb()
    rw = RobotWrapper.BuildFromURDF(urdf, mesh)
    rw.model.referenceConfigurations["standing"] = robot.go_neutral()
    q0 = rw.model.referenceConfigurations["standing"].copy()
    x0 = np.concatenate([q0, np.zeros(rw.model.nv)])
    model = rw.model

    gait = SimpleQuadrupedalGaitProblem(model, FOOT["FL"], FOOT["FR"], FOOT["RL"], FOOT["RR"])
    ALL = [FOOT[k] for k in ("FL", "FR", "RL", "RR")]
    order = [FOOT[k.strip()] for k in args.order.split(",")]
    dir_sign = -1.0 if args.direction == "right" else 1.0
    direction = (0.0, dir_sign)
    # left = exact Y-mirror of right: flip direction (+Y) AND swap the L<->R feet in
    # the swing order (FR,RL,RR,FL -> FL,RR,RL,FR), same recipe as the baseline driver.
    if args.direction == "left":
        LR = {"FL_foot_link": "FR_foot_link", "FR_foot_link": "FL_foot_link",
              "RL_foot_link": "RR_foot_link", "RR_foot_link": "RL_foot_link"}
        order = [LR[n] for n in order]
    timeStep = 0.01
    fid = {n: model.getFrameId(n) for n in ALL}
    # speed = stride x cadence scaling (same as the baseline / reorder drivers):
    # split R=speed/0.05 between stride (foot displacement) and cadence (fewer knots).
    R = args.speed / 0.05
    cadence = R ** args.cadence_share
    stride = args.stride * R ** (1.0 - args.cadence_share)
    stepknots = max(8, int(args.stepknots / cadence + 0.5))
    preshift = max(3, int(args.preshift / cadence + 0.5))
    hold = max(3, int(args.hold / cadence + 0.5))

    # Custom state-regularisation vector passed to the (patched) library methods:
    # base-orientation lowered to cap trunk tilt ~5 deg, hip & thigh/calf kept at
    # stock 50 so legs stay natural (no 内八, no crouch). Layout matches stock:
    # [0]*3 base-pos + [ori]*3 + [hip,thigh,calf]x4 + [10]*6 base-vel + [1]*joints-vel.
    nv = model.nv
    joint_ws = []
    for _ in range(4):                      # FL, FR, RL, RR
        joint_ws += [args.hip_reg, args.joint_reg, args.joint_reg]
    STATE_W = np.array([0.0] * 3 + [args.base_ori_reg] * 3 + joint_ws
                       + [10.0] * 6 + [1.0] * (nv - 6))

    def build(x0):
        q = x0[: model.nq]
        pinocchio.forwardKinematics(model, gait.rdata, q)
        pinocchio.updateFramePlacements(model, gait.rdata)
        feetPos = {n: gait.rdata.oMf[fid[n]].translation.copy() for n in ALL}
        com0 = pinocchio.centerOfMass(model, gait.rdata, q)
        comHoriz0 = com0[:2].copy()
        comHeight = com0[2].item()

        def supportCoM(supportPos):
            tri = np.array([p[:2] for p in supportPos])
            centroid = tri.mean(axis=0)
            tri = centroid + (1.0 - args.margin) * (tri - centroid)   # shrink
            xy = closest_point_in_triangle(comHoriz0, tri[0], tri[1], tri[2])
            return np.array([xy[0], xy[1], comHeight])

        loco = []
        for swing in order:
            supportNames = [n for n in ALL if n != swing]
            supportPos = [feetPos[n] for n in supportNames]
            comT = supportCoM(supportPos)
            stepHeight = args.front_stepheight if swing in FRONT else args.hind_stepheight
            pre = [gait.createModel(timeStep=timeStep, footContacts=ALL, comTask=comT,
                                    constraint=args.constraint, stateWeights=STATE_W)
                   for _ in range(preshift)]
            step = gait.createFootstepModels(comT.copy(), [feetPos[swing].copy()],
                                             stride, stepHeight, timeStep,
                                             stepknots, supportNames, [swing],
                                             constraint=args.constraint, direction=direction,
                                             stateWeights=STATE_W)
            post = [gait.createModel(timeStep=timeStep, footContacts=ALL, comTask=comT,
                                     constraint=args.constraint, stateWeights=STATE_W)
                    for _ in range(hold)]
            loco += pre + step + post
            feetPos[swing] = feetPos[swing] + np.array([direction[0], direction[1], 0.0]) * stride
        return crocoddyl.ShootingProblem(x0, loco[:-1], loco[-1])

    solvers = []
    for i in range(args.cycles):
        problem = build(x0)
        solver = (crocoddyl.SolverIntro if args.constraint else crocoddyl.SolverFDDP)(problem)
        xs = [x0] * (solver.problem.T + 1)
        us = solver.problem.quasiStatic([x0] * solver.problem.T)
        solver.solve(xs, us, args.maxiter, False)
        print(f"cycle {i}: T={solver.problem.T} iters={solver.iter} "
              f"stop={solver.stop:.2e} cost={solver.cost:.3f} feas={solver.isFeasible}")
        solvers.append(solver)
        x0 = solver.xs[-1]

    jn = [model.names[i] for i in range(2, model.njoints)]
    fields = (["root_pos_x", "root_pos_y", "root_pos_z",
               "root_rot_x", "root_rot_y", "root_rot_z", "root_rot_w"] + jn
              + ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"])
    bfid = model.getFrameId("Base_link")
    tag = "" if args.direction == "right" else "left_"
    out = args.out or f"trajectory_tripod_com_{tag}tilt_v{args.speed:.2f}.csv"
    n = 0
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for solver in solvers:
            for x in solver.xs:
                q = x[: model.nq]
                pinocchio.forwardKinematics(model, rw.data, q)
                pinocchio.updateFramePlacements(model, rw.data)
                bp = rw.data.oMf[bfid]
                quat = pinocchio.SE3ToXYZQUAT(bp)[3:7]
                row = {"root_pos_x": bp.translation[0], "root_pos_y": bp.translation[1],
                       "root_pos_z": bp.translation[2], "root_rot_x": quat[0],
                       "root_rot_y": quat[1], "root_rot_z": quat[2], "root_rot_w": quat[3]}
                for j, val in enumerate(q[7:]):
                    row[jn[j]] = val
                for fj in ("FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"):
                    row[fj] = 0.0
                w.writerow(row)
                n += 1
    print(f"\nwrote {out}: {n} rows x {len(fields)} cols  (order {args.order}, "
          f"margin {args.margin}, base-ori {args.base_ori_reg} ~5deg-tilt, "
          f"front {args.front_stepheight*100:.0f}cm / hind {args.hind_stepheight*100:.0f}cm)")


if __name__ == "__main__":
    main()
