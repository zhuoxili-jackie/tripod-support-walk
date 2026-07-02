"""Per-cycle foot-spacing diagnosis for the single-leg-swing (tripod) gait.

Companion to ``_verify_tripod_com.py``. That tool checks the CoM / pitch story;
this one checks the *lateral tracking* story raised in §0.5-D of the continuation
doc: does the horizontal distance between the two hind feet (RL-RR) and the two
front feet (FL-FR) stay put, or does it monotonically narrow cycle after cycle?
That narrowing is the symptom of the solver missing its per-step landing target,
with the residual accumulating through the warm-start chain (each cycle starts
from ``x0 = previous solver.xs[-1]``).

Method (read-only, pure FK):
  1. The CSV is N gait cycles of equal length concatenated (default N=8, matching
     the drivers). Split the rows into N equal blocks; the END of a cycle is that
     block's last frame -- the terminal double-support where all feet are planted,
     so spacing is compared at a consistent gait phase.
  2. FK that frame -> horizontal (x,y) of each foot; report |RL-RR| and |FL-FR|.
  3. Print a per-cycle table + the cycle0->last delta (like §0.5-D) and whether
     the spacing is drifting monotonically. The 0.05 baseline's ~1cm/8-cycle
     wobble is the noise floor to beat.

This reproduces the §0.5-D table exactly for the baseline
(RL-RR 0.4396 -> 0.4292, FL-FR 0.4459 -> 0.4509).

Usage:
    python _verify_foot_spacing.py <csv> [<csv> ...] [--cycles N]
Multiple CSVs -> a side-by-side summary (handy for old-vs-new comparison).
"""
import sys
import numpy as np
import pinocchio
from pcb_v2.pcbWrapper import pcb

robot = pcb()
model, rdata = robot.model, robot.data
FEET = ["FL_foot_link", "FR_foot_link", "RL_foot_link", "RR_foot_link"]
fid = {n: model.getFrameId(n) for n in FEET}


def spacing_at(row):
    q = row[: model.nq].copy()
    pinocchio.forwardKinematics(model, rdata, q)
    pinocchio.updateFramePlacements(model, rdata)
    xy = {n: rdata.oMf[fid[n]].translation[:2].copy() for n in FEET}
    rl_rr = float(np.linalg.norm(xy["RL_foot_link"] - xy["RR_foot_link"]))
    fl_fr = float(np.linalg.norm(xy["FL_foot_link"] - xy["FR_foot_link"]))
    return rl_rr, fl_fr


def analyze(csv_path, n_cycles):
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)
    if len(data) % n_cycles != 0:
        print(f"\n=== {csv_path}: {len(data)} rows NOT divisible by {n_cycles} "
              f"cycles -- pass the right --cycles ===")
    per = len(data) // n_cycles
    ends = [(c + 1) * per - 1 for c in range(n_cycles)]
    rows = [(k, *spacing_at(data[k])) for k in ends]
    print(f"\n=== {csv_path}: {len(data)} rows, {n_cycles} cycles x {per} frames ===")
    print("%-6s %-8s %-12s %-12s" % ("cycle", "frame", "RL-RR (m)", "FL-FR (m)"))
    for c, (k, rr, ff) in enumerate(rows):
        print("%-6d %-8d %-12.4f %-12.4f" % (c, k, rr, ff))
    rr0, ff0 = rows[0][1], rows[0][2]
    rrN, ffN = rows[-1][1], rows[-1][2]
    d_rr, d_ff = rrN - rr0, ffN - ff0
    rr_series = [r[1] for r in rows]
    ff_series = [r[2] for r in rows]
    # monotone within a 0.2mm tolerance so tiny numerical wiggles don't hide a trend
    rr_mono = all(b - a <= 2e-4 for a, b in zip(rr_series[:-1], rr_series[1:]))
    ff_mono = all(b - a <= 2e-4 for a, b in zip(ff_series[:-1], ff_series[1:]))
    print("-" * 44)
    print("RL-RR: %.4f -> %.4f  (%+.4f m%s)" %
          (rr0, rrN, d_rr, ", monotonically narrowing" if rr_mono else ""))
    print("FL-FR: %.4f -> %.4f  (%+.4f m%s)" %
          (ff0, ffN, d_ff, ", monotonically narrowing" if ff_mono else ""))
    return csv_path, rr0, rrN, d_rr, ff0, ffN, d_ff


def main():
    argv = sys.argv[1:]
    n_cycles = 8
    if "--cycles" in argv:
        i = argv.index("--cycles")
        n_cycles = int(argv[i + 1])
        del argv[i:i + 2]
    paths = argv or ["trajectory_walking_sideways.csv"]
    summary = [analyze(p, n_cycles) for p in paths]
    if len(summary) > 1:
        print("\n\n================ SUMMARY (cycle0 -> last) ================")
        for p, rr0, rrN, drr, ff0, ffN, dff in summary:
            name = p.rsplit("/", 1)[-1]
            print("%-42s  RL-RR %6.4f->%6.4f %+0.4f   FL-FR %6.4f->%6.4f %+0.4f" %
                  (name, rr0, rrN, drr, ff0, ffN, dff))


if __name__ == "__main__":
    main()
