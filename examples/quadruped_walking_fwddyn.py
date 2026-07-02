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
    python quadruped_walking_fwddyn.py                  # default: 0.05 m/s -> trajectory_walking_sideways.csv
    python quadruped_walking_fwddyn.py --speed 0.1       # -> trajectory_walking_sideways_sc_v0.10.csv

``--speed`` sets the average sideways speed in m/s (v = stepLength /
T_step_cycle, with T_step_cycle = (2*supportKnots + 4*stepKnots) * timeStep the
wall-clock duration of one full 4-swing gait cycle).

Higher speed is reached by scaling BOTH stride (stepLength) AND cadence (step
frequency), instead of the old behaviour of only inflating stepLength -- which
made the steps huge and the feet spacing narrow cycle after cycle (§0.5-D). The
speed ratio R = speed/0.05 is split geometrically between the two dimensions via
``--cadence-share`` alpha in [0,1]: cadenceFactor = R**alpha and stride takes the
rest, so cadenceFactor * strideFactor = R. Cadence is realised by shrinking
stepKnots/supportKnots (shorter T_step_cycle); ``timeStep`` stays 0.01 always,
because the 100 Hz CSV contract (RUN_PIPELINE §4) forbids touching it. stepLength
is then recomputed from the ACTUAL integer-rounded cycle duration, so the average
speed lands exactly on target.

Cycle 0 is a SYMMETRIC gentle ramp: the stock ``firstStep`` asymmetry (halving
only the first two swings) is disabled and instead all four feet take a
FIRST_CYCLE_STRIDE_FRAC x stepLength step in cycle 0. This removes a cold-start
tremble of the RL support foot during the FR/RR swings of cycle 0 (present in the
stock gait, worse at higher speed) with no CoM/pitch regression, while still
starting from the neutral standing pose. Because cycle 0 seeds the chain, this
changes the WHOLE trajectory, not just cycle 0: at 0.05 the steady-state feet
settle at the near-nominal ~0.41 m track width instead of the stock baseline's
slightly splayed ~0.44 m (the splay was itself a symptom of the asymmetric cold
start), so the output is genuinely different from -- not a byte-shift of --
trajectory_walking_sideways.csv. The frozen baseline file is never overwritten
here; outputs go to trajectory_walking_sideways_sc_v*.csv.
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
parser.add_argument("--speed", type=float, default=0.05,
                     help="average sideways speed in m/s (default 0.05, matches the reference CSV)")
parser.add_argument("--cadence-share", type=float, default=0.5,
                     help="how the speed-up above 0.05 is split between cadence and "
                          "stride, alpha in [0,1]: cadenceFactor=R**alpha, "
                          "strideFactor=R**(1-alpha), R=speed/0.05. 0=pure stride "
                          "(old --speed behaviour), 1=pure cadence, 0.5=equal "
                          "geometric split (default). No effect at speed 0.05.")
parser.add_argument("--out", type=str, default=None,
                     help="output CSV path (default: derived from --speed)")
args = parser.parse_args()

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
# The stock ``firstStep`` ramp halves ONLY the first two swings (rh, rf = FL, RL
# here) while the other two (lh, lf = FR, RR) take a full step -- an ASYMMETRIC
# cold start. That asymmetry leaves the RL support foot in a weakly-constrained
# pose that the cold FDDP solve resolves with a visible tremble during the FR/RR
# swings of cycle 0 (gone by steady state). We disable it and instead ramp cycle
# 0 SYMMETRICALLY (all four feet at FIRST_CYCLE_STRIDE_FRAC x stepLength below):
# that kills the tremble (RL foot travel 21 mm -> 0.1 mm) with NO CoM/pitch
# regression and still starts from the neutral standing pose. Driver-only.
gait.firstStep = False

# --- gait parameters (baseline: 8 cycles x 165 rows = 1320, like the reference CSV) --
N_CYCLES = 8
FIRST_CYCLE_STRIDE_FRAC = 0.5  # symmetric gentle ramp for cycle 0 (all 4 feet half-step)
timeStep = 0.01              # 100 Hz -- FIXED by the CSV contract, never scaled
BASE_SPEED = 0.05            # the validated baseline speed
BASE_STEP_KNOTS = 35         # baseline swing-phase nodes
BASE_SUPPORT_KNOTS = 10      # baseline double-support nodes
MIN_STEP_KNOTS = 8           # floor: keep the swing arc smooth enough to track
MIN_SUPPORT_KNOTS = 3        # floor: keep a real double-support / CoM-shift window
stepHeight = 0.10            # swing clearance
direction = (0.0, -1.0)      # walk -Y (sideways), matching the reference

# Joint stride x cadence scaling (§0.5-D): split R = speed/BASE_SPEED geometrically
# between cadence (via fewer knots -> shorter cycle) and stride (via stepLength).
R = args.speed / BASE_SPEED
alpha = args.cadence_share
cadenceFactor = R ** alpha
stepKnots = max(MIN_STEP_KNOTS, int(BASE_STEP_KNOTS / cadenceFactor + 0.5))
supportKnots = max(MIN_SUPPORT_KNOTS, int(BASE_SUPPORT_KNOTS / cadenceFactor + 0.5))
T_step_cycle = (2 * supportKnots + 4 * stepKnots) * timeStep   # actual cycle duration
stepLength = args.speed * T_step_cycle                         # hits target avg speed exactly

# Always write the sc_ ("stride x cadence") name; never auto-overwrite the frozen
# baseline trajectory_walking_sideways.csv. The sc_ family carries the joint
# scaling AND the symmetric cycle-0 ramp (jitter fix); the original stock baseline
# stays committed as-is. Pass --out to override.
output_path = args.out or f"trajectory_walking_sideways_sc_v{args.speed:.2f}.csv"

swing_window = stepKnots * timeStep
print(f"speed={args.speed} m/s  R={R:.3f}  cadence-share alpha={alpha}")
print(f"  cadenceFactor={cadenceFactor:.3f} -> stepKnots={stepKnots} "
      f"supportKnots={supportKnots}  T_step_cycle={T_step_cycle:.3f}s "
      f"(freq x{1.60 / T_step_cycle:.3f} vs baseline 1.60s)")
print(f"  stepLength={stepLength:.4f} m  (stride x{stepLength / 0.08:.3f} vs baseline 0.08m)"
      f"  swing_window={swing_window:.3f}s  peak_tip_speed~{stepLength / swing_window:.3f} m/s")

solvers = []
for i in range(N_CYCLES):
    # cycle 0 = symmetric gentle ramp from rest; cycles 1+ = full stride
    cyc_stepLength = FIRST_CYCLE_STRIDE_FRAC * stepLength if i == 0 else stepLength
    problem = gait.createWalkingProblem(
        x0, cyc_stepLength, stepHeight, timeStep, stepKnots, supportKnots,
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
