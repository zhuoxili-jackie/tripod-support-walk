"""Per-swing diagnosis: for each single-leg-swing segment, report the lifted
foot, the actual mean CoM (x,y), the 3-support-triangle centroid, and how many
frames the CoM stays inside the triangle. Read-only."""
import sys
import numpy as np
import pinocchio
from pcb_v2.pcbWrapper import pcb

csv_path = sys.argv[1] if len(sys.argv) > 1 else "trajectory_tripod_f004.csv"
data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)

robot = pcb()
model, rdata = robot.model, robot.data
FEET = ["FL_foot_link", "FR_foot_link", "RL_foot_link", "RR_foot_link"]
fid = {n: model.getFrameId(n) for n in FEET}


def point_in_tri(p, a, b, c):
    v0, v1, v2 = b - a, c - a, p - a
    d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
    d20, d21 = v2 @ v0, v2 @ v1
    den = d00 * d11 - d01 * d01
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    u = 1.0 - v - w
    return (u >= 0 and v >= 0 and w >= 0), min(u, v, w)


def frame_state(row):
    q = row[:model.nq].copy()
    pinocchio.forwardKinematics(model, rdata, q)
    pinocchio.updateFramePlacements(model, rdata)
    z = {n: rdata.oMf[fid[n]].translation[2] for n in FEET}
    xy = {n: rdata.oMf[fid[n]].translation[:2].copy() for n in FEET}
    com = pinocchio.centerOfMass(model, rdata, q)[:2].copy()
    return z, xy, com


nominal = frame_state(data[0])[0]
# classify each frame's lifted foot
labels = []
for row in data:
    z, xy, com = frame_state(row)
    lifted = [n for n in FEET if z[n] - nominal[n] > 0.02]
    labels.append(lifted[0] if len(lifted) == 1 else None)

# group contiguous swing segments
segs = []
i = 0
while i < len(labels):
    if labels[i] is None:
        i += 1
        continue
    j = i
    while j < len(labels) and labels[j] == labels[i]:
        j += 1
    segs.append((labels[i], i, j))
    i = j

print(f"CSV {csv_path}: {len(data)} rows, {len(segs)} swing segments\n")
print("%-14s %6s  %-18s %-18s %-16s %s" %
      ("lifted", "frames", "mean CoM(x,y)", "tri centroid(x,y)",
       "inside/frames", "worst margin"))
tot_in = tot = 0
for name, a, b in segs:
    coms, ins, marg = [], 0, 1e9
    cx = cy = 0.0
    for k in range(a, b):
        z, xy, com = frame_state(data[k])
        support = [n for n in FEET if n != name]
        inside, m = point_in_tri(com, xy[support[0]], xy[support[1]], xy[support[2]])
        ins += inside
        marg = min(marg, m)
        coms.append(com)
        cen = np.mean([xy[s] for s in support], axis=0)
        cx, cy = cen
    mc = np.mean(coms, axis=0)
    tot_in += ins
    tot += (b - a)
    print("%-14s %6d  (%+.3f,%+.3f)   (%+.3f,%+.3f)    %5d/%-8d  %+.4f" %
          (name, b - a, mc[0], mc[1], cx, cy, ins, b - a, marg))
print(f"\nTOTAL inside: {tot_in}/{tot}")
