#!/usr/bin/env python3
"""Step 3 (§0.6-3) + v1.11 knock-knee fix (2026-07-03 per user, option B):
tripod gait that leans the CoM toward each support triangle, caps the trunk tilt
at ~5 deg, AND pins the hips near neutral to remove the human-like knock-knee
(内八), accepting that hind swings then tip a little more (a later RL policy
rewards the rest).

Why the hips splay in the first place: a single-leg-swing gait (walking/tripod)
has NO diagonal partner to self-balance, so the support-leg hips actively adduct
to pull the CoM toward the support side. Trotting (diagonal pairs) never needs
this, which is why trotting hips stay ~0 while stock walking/tripod splay 8-19
deg. The adduction is a lateral-balance NECESSITY, not a regulariser artefact --
so merely bumping the hip weight a little does nothing (50->200 only moved 14->13
deg). The fix here is to pin the hips HARD (``--hip-reg`` 300, effective weight
1e1*300**2 ~ 1e6, on par with the comTrack) so the hips stay ~7 deg (trotting
level); the lost lateral balance is NOT recoverable within the ~5 deg tilt cap,
so the hind swings need a few N more diagonal-front pull than before (worst pull
~19 N vs ~11 N; still a moderate tip the RL policy absorbs). Body ROLL is NOT
opened to buy the balance back: doing so injects a ~5 deg world-yaw drift for
little gain, so pitch/roll/yaw stay stiff and the trunk stays clean.

How it is enforced: the library createModel/createFootstepModels take an optional
``stateWeights`` vector (default None = stock, so trot/walking are unchanged).
This driver passes a vector that
  * lowers base-orientation 500 -> ``--base-ori-reg`` (250 keeps tilt ~5 deg), and
  * RAISES the hip-POSITION weight to ``--hip-reg`` (300) while keeping thigh/calf
    at the stock ``--joint-reg`` (50). Pinning the hip transfers a little
    deformation to the front thighs (support bend ~57->67 deg) but no deep crouch.
  * RAISES the hip-VELOCITY weight to ``--hipvel-reg`` (25). A stiff hip-position
    cost with the stock velocity weight is under-damped and makes the WHOLE BODY
    tremble at ~18 Hz (base 2nd-diff 5x, hips 15x, solve 300 iters). Damping only
    the hip velocities kills the tremble (smoother than stock, ~20 iters) while
    leaving thigh/calf velocities free so the foot lift is preserved.

Feet lift: front feet swing to ``--front-stepheight`` (~15 cm actual peak), hind
to ``--hind-stepheight`` (~10 cm) -- both raised from the old 0.12/0.115 to offset
the small lift trim the hip-velocity damping causes.

Run (examples/, croco310, build_conda on PYTHONPATH; library already cp'd there):
    python quadruped_tripod_com.py                 # hip~7deg, tilt<4deg, front 14/hind 9cm
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
    ap.add_argument("--front-stepheight", type=float, default=0.155,
                    help="front-foot swing clearance param (~15cm actual peak). Raised "
                         "from 0.12 to offset the hip-velocity damping, which trims lift")
    ap.add_argument("--hind-stepheight", type=float, default=0.165,
                    help="hind-foot swing clearance param (~10cm actual peak). Raised "
                         "from 0.115 to offset the hip-velocity damping")
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
    ap.add_argument("--hip-reg", type=float, default=300.0,
                    help="hip (lateral) weight (stock 50). 300 pins the hips ~7 deg "
                         "(trotting level) to kill the knock-knee/内八; 50 = stock = "
                         "splays 14 deg (balance adduction). Higher = less splay but "
                         "more hind-swing tip (no lateral balance left)")
    ap.add_argument("--joint-reg", type=float, default=50.0,
                    help="thigh+calf weight (stock 50). Keep at 50 -> no deep crouch")
    ap.add_argument("--hipvel-reg", type=float, default=25.0,
                    help="hip joint-VELOCITY weight (stock 1). 25 damps the ~18 Hz body "
                         "trembling that the stiff --hip-reg 300 otherwise induces "
                         "(under-damped stiff cost). Only the 4 hip velocities are "
                         "raised -> thigh/calf stay free so the foot lift is preserved")
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
    # base-orientation lowered to cap trunk tilt ~5 deg, HIP POS RAISED to --hip-reg
    # (300) to pin the hips ~7 deg (kill the 内八), thigh/calf pos kept at stock 50 (no
    # deep crouch), and HIP VELOCITY raised to --hipvel-reg (25) to damp the trembling
    # the stiff hip-pos cost otherwise induces (thigh/calf velocity stays 1 so the foot
    # lift survives). Layout: [0]*3 base-pos + [ori]*3 + [hip,thigh,calf]x4 pos +
    # [10]*6 base-vel + [hipvel,1,1]x4 joints-vel.
    nv = model.nv
    joint_ws = []
    joint_vel_ws = []
    for _ in range(4):                      # FL, FR, RL, RR
        joint_ws += [args.hip_reg, args.joint_reg, args.joint_reg]
        joint_vel_ws += [args.hipvel_reg, 1.0, 1.0]
    STATE_W = np.array([0.0] * 3 + [args.base_ori_reg] * 3 + joint_ws
                       + [10.0] * 6 + joint_vel_ws)

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
          f"margin {args.margin}, hip-reg {args.hip_reg} (pin hips ~7deg), "
          f"hipvel-reg {args.hipvel_reg} (anti-tremble), base-ori {args.base_ori_reg}, "
          f"front {args.front_stepheight*100:.0f}cm / hind {args.hind_stepheight*100:.0f}cm param)")


if __name__ == "__main__":
    main()
