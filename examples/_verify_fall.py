#!/usr/bin/env python3
"""Will-it-fall? judge for a tripod-support reference CSV (READ-ONLY).

Why CoM-in-triangle is not enough
---------------------------------
``_verify_tripod_com.py`` only asks whether the CoM's vertical projection sits
inside the support polygon -- a purely *static* test (assumes zero acceleration
and, implicitly, that the feet can pull as well as push). The real robot must
also obey:

  * contacts are **unilateral** (a foot can only push, ``f_z >= 0``) and live
    inside a **friction cone** (``|f_x|,|f_y| <= mu f_z``). If balancing the
    body would need a foot to pull the ground down, it physically can't -- the
    robot tips. This is the failure mode the loads probe found on this robot:
    with the CoM at x=+0.06 and the front feet up on a 0.8 m box, lifting a hind
    leg leaves the diagonal front foot needing a *negative* vertical force.
  * **dynamics**: CoM acceleration and the rate of change of centroidal angular
    momentum shift the effective balance point (ZMP/CoP) away from the CoM. At
    0.05 m/s this is nearly quasi-static (ZMP ~= CoM), but the judge keeps the
    dynamic terms so it stays honest at higher speed / in a physics sim.
  * even a momentarily out-of-support CoM might be recoverable if inertia hasn't
    carried the body past the tipping "point of no return" yet -- a **tip-over**
    estimate answers "how fast would it fall".

So this script reports four layers, cheapest -> strictest, per support phase:

  1. static margin [m]  : signed distance from CoM(x,y) to the support-polygon
                          boundary (inside positive). Metres, so it does not get
                          distorted by triangle size the way a barycentric
                          coordinate does (see §0.5-E of the working doc).
  2. contact-force LP   : is there ANY set of physically legal contact forces
     (unilateral + friction cone) that produces this trajectory's accelerations
     (force AND moment balance about the CoM, with the measured a_com and
     dL/dt)? This is THE physical feasibility test -- it needs no co-planar-feet
     assumption, unlike ZMP. Reports feasibility + the minimum cone violation
     (N): 0 means feasible, >0 is the worst "pull / exceed-friction" force
     needed. A quasi-static vertical-load solve (the loads-probe special case)
     is also printed per foot, so the numbers tie back to the probe: any
     negative f_z there == a foot pulling the ground == a tip.
  3. ZMP margin [m]     : signed distance from the (dynamic) ZMP to the support
     polygon. Quasi-static it ~= the static margin; the delta grows with speed.
     NOTE: the feet here are NOT co-planar (front on a box, hind on the floor),
     so a single-plane ZMP is an approximation -- layer 2 (the LP) is the
     rigorous dynamic-feasibility judge; ZMP is kept as an intuitive metre-scale
     number and a hook for future higher speeds.
  4. tip-over [deg]     : when the CoM is outside support, model the body as
     rotating about the nearest support edge; report the tipping angular accel
     and the angle accumulated over the phase vs the angle to the point of no
     return -- a "how badly / how fast" number.

Verdict per phase: LP infeasible (cone violation above a small tol) => FALL;
feasible but static margin below ~2-3 cm => RISKY; else OK.

Usage (inside examples/, croco310 env, build_conda on PYTHONPATH):
    python _verify_fall.py trajectory_walking_sideways_sc_v0.05.csv
    python _verify_fall.py a.csv b.csv          # summary line per file
"""
import sys

import numpy as np
import pinocchio
from scipy.optimize import linprog, nnls
from scipy.signal import savgol_filter

from pcb_v2.pcbWrapper import pcb

# --- constants ---------------------------------------------------------------
G = 9.81
MU = 0.7                 # friction coeff, matches library FrictionCone(.,0.7,4,False)
DT = 0.01                # 100 Hz CSV contract
SWING_THRESH = 0.04      # foot swinging if it rose > 4 cm above its LOCAL support
WIN_HALF = 60            # half-window (frames) for the per-foot support baseline
TRANS_MAX_FRAMES = 5     # a <3-support run longer than this is a real (unstable)
                         # 2-foot phase, not a swing-switch blip -> judged, not skipped
CONE_TOL = 1.0           # N: LP cone violation below this counts as feasible
RISKY_MARGIN = 0.03      # m: feasible but static margin below this = RISKY
FEET = ["FL_foot_link", "FR_foot_link", "RL_foot_link", "RR_foot_link"]


# --- geometry ----------------------------------------------------------------
def polygon_ccw(pts):
    """Order 2D points counter-clockwise (they form a convex support polygon)."""
    pts = np.asarray(pts, dtype=float)
    ctr = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - ctr[1], pts[:, 0] - ctr[0])
    return pts[np.argsort(ang)]


def signed_margin(p_xy, support_pts):
    """Signed distance (m) from point p to the support polygon boundary.

    Inside -> positive (distance to the nearest edge); outside -> negative
    (the deepest edge penetration). For a triangle this is exactly "CoM to the
    three edges, min", which is the metre-scale version of the barycentric
    margin used by _verify_tripod_com.
    """
    poly = polygon_ccw(support_pts)
    n = len(poly)
    min_line = 1e9      # min signed distance to the edge SUPPORTING LINES (inside +)
    min_seg = 1e9       # min Euclidean distance to the edge SEGMENTS (boundary)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        e2 = e @ e
        elen = np.sqrt(e2)
        # inward normal of a CCW polygon = edge rotated +90 deg
        nx, ny = -e[1] / elen, e[0] / elen
        min_line = min(min_line, (p_xy[0] - a[0]) * nx + (p_xy[1] - a[1]) * ny)
        t = np.clip((p_xy - a) @ e / e2, 0.0, 1.0)
        q = a + t * e
        min_seg = min(min_seg, float(np.hypot(*(p_xy - q))))
    # inside: distance to nearest edge == distance to boundary (min_line >= 0).
    # outside: min_line under-estimates near a vertex, so use the true Euclidean
    # distance to the boundary, negated.
    return min_line if min_line >= 0 else -min_seg


def detect_support(z_series):
    """Per-frame support set, robust to the reference's whole-body drift.

    The soft ContactModel3D lets every foot height creep -- up to ~15 cm over a
    fast run -- so any fixed or global threshold wrongly flags drifted stance
    feet as lifted. Each foot is instead compared to its OWN LOCAL support
    height: the minimum of its z over a +-WIN_HALF window, which tracks the drift
    no matter how large. A foot that rises SWING_THRESH above that local floor is
    swinging.

    Assumption (true for any real quadruped gait, and the reason a global-min
    fallback is deliberately NOT used -- it re-breaks under large drift): every
    foot touches down at least once inside each +-WIN_HALF window. That holds
    when each foot swings once per cycle and the window (2*WIN_HALF+1 frames)
    spans at least one stance interval. It is NOT meant for a foot deliberately
    held airborne for a whole window. Returns a support-foot tuple per frame.
    """
    N = len(next(iter(z_series.values())))
    support = []
    for t in range(N):
        lo, hi = max(0, t - WIN_HALF), min(N, t + WIN_HALF + 1)
        sup = tuple(n for n in FEET
                    if z_series[n][t] - z_series[n][lo:hi].min() <= SWING_THRESH)
        support.append(sup)
    return support


def nearest_edge_dist(p_xy, support_pts):
    """(distance, foot-of-perpendicular) from p to the polygon boundary (>=0)."""
    poly = polygon_ccw(support_pts)
    n = len(poly)
    best_d, best_q = 1e9, None
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        t = np.clip((p_xy - a) @ e / (e @ e), 0.0, 1.0)
        q = a + t * e
        d = np.hypot(*(p_xy - q))
        if d < best_d:
            best_d, best_q = d, q
    return best_d, best_q


# --- contact-force feasibility LP -------------------------------------------
def lp_cone_violation(support_p3, c, Rnet, tau):
    """Minimum friction-cone violation (N) over legal-force reconstructions.

    Variables: 3D force f_i at every support foot, plus a scalar slack z that
    every pyramid facet is allowed to exceed. Minimise z subject to exact force
    and moment balance:
        sum f_i               = Rnet            (= M*a_com - M*g_vec)
        sum (p_i - c) x f_i   = tau             (= dL/dt about the CoM)
    and the pyramid friction cone (which implies unilaterality f_iz>=0):
        +-f_ix - mu f_iz <= z ,  +-f_iy - mu f_iz <= z .

    z<=0  -> feasible, |z| is the in-cone margin (N).
    z>0   -> infeasible, z is the worst force by which some facet is violated
             (a foot pulling the ground and/or slipping). Returns (z, forces).
    """
    n = len(support_p3)
    nx = 3 * n + 1                      # [f_0x,f_0y,f_0z, ..., z]
    zc = nx - 1                         # index of the slack
    cobj = np.zeros(nx)
    cobj[zc] = 1.0

    # equality: 3 force + 3 moment rows
    A_eq = np.zeros((6, nx))
    b_eq = np.concatenate([Rnet, tau])
    for i, p in enumerate(support_p3):
        r = p - c
        fx, fy, fz = 3 * i, 3 * i + 1, 3 * i + 2
        # force balance
        A_eq[0, fx] = 1.0
        A_eq[1, fy] = 1.0
        A_eq[2, fz] = 1.0
        # moment r x f = (ry fz - rz fy, rz fx - rx fz, rx fy - ry fx)
        A_eq[3, fz] += r[1];  A_eq[3, fy] += -r[2]
        A_eq[4, fx] += r[2];  A_eq[4, fz] += -r[0]
        A_eq[5, fy] += r[0];  A_eq[5, fx] += -r[1]

    # inequality: 4 pyramid facets per foot, each minus the slack z
    A_ub = np.zeros((4 * n, nx))
    b_ub = np.zeros(4 * n)
    for i in range(n):
        fx, fy, fz = 3 * i, 3 * i + 1, 3 * i + 2
        rows = [
            (+1, fx), (-1, fx), (+1, fy), (-1, fy),
        ]
        for j, (sgn, idx) in enumerate(rows):
            row = 4 * i + j
            A_ub[row, idx] = sgn
            A_ub[row, fz] = -MU
            A_ub[row, zc] = -1.0

    bounds = [(None, None)] * nx       # forces and slack all free
    res = linprog(cobj, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq,
                  bounds=bounds, method="highs")
    if not res.success:
        return None, None              # balance itself unsatisfiable (shouldn't happen)
    z = res.x[zc]
    forces = res.x[:3 * n].reshape(n, 3)
    return z, forces


def vertical_loads(support_p3, c, Rz, tau_x, tau_y):
    """Quasi-static-style vertical foot forces (the loads-probe special case).

    Solve only for the vertical components f_iz that carry the net vertical
    force Rz and cancel the horizontal moments about the CoM:
        sum f_iz              = Rz
        sum (py_i - cy) f_iz  = tau_x
        sum (px_i - cx) f_iz  = -tau_y
    Unique for 3 supports; min-norm least squares for 4. A negative f_iz means
    that foot must pull the ground down == the robot tips (this equals
    mg * (min barycentric coord) in the quasi-static limit, tying to
    _verify_tripod_com and probe_loads).
    """
    n = len(support_p3)
    A = np.zeros((3, n))
    # rows: sum f=Rz ; sum (py-cy) f = (rxf)_x = tau_x ; sum -(px-cx) f = (rxf)_y = tau_y
    b = np.array([Rz, tau_x, tau_y])
    for i, p in enumerate(support_p3):
        A[0, i] = 1.0
        A[1, i] = p[1] - c[1]
        A[2, i] = -(p[0] - c[0])
    if n == 3:
        f = np.linalg.solve(A, b)                 # unique (ties to probe_loads)
    else:
        # 4-foot support is statically indeterminate; pick the most even
        # non-negative split (tiny Tikhonov reg toward equal loads) for display.
        lam = 0.05
        A_aug = np.vstack([A, lam * np.eye(n)])
        b_aug = np.concatenate([b, np.zeros(n)])
        f, _ = nnls(A_aug, b_aug)
    return f


def zmp_point(c, a_com, dL, z_ground):
    """Ground ZMP/CoP on the plane z = z_ground (single-plane approximation).

    Balances the gravito-inertial wrench: net contact force
    R = M(a_com - g) acts at the ZMP, and its moment plus the free moment dL
    must cancel the gravito-inertial moment about the ground point. With feet at
    different heights this single plane is only approximate; layer 2 is the
    rigorous test. Returns (zmp_x, zmp_y).
    """
    # net upward reaction force
    Rz = M * (a_com[2] + G)
    if abs(Rz) < 1e-6:
        return np.array([c[0], c[1]])
    # standard Kajita ZMP with angular-momentum term, ground at z_ground
    zx = (M * (a_com[2] + G) * c[0] - M * a_com[0] * (c[2] - z_ground) - dL[1]) / Rz
    zy = (M * (a_com[2] + G) * c[1] - M * a_com[1] * (c[2] - z_ground) + dL[0]) / Rz
    return np.array([zx, zy])


# --- load model + CSV --------------------------------------------------------
robot = pcb()
model, rdata = robot.model, robot.data
M = sum(I.mass for I in model.inertias)
fid = {n: model.getFrameId(n) for n in FEET}


def kinematics(q):
    pinocchio.forwardKinematics(model, rdata, q)
    pinocchio.updateFramePlacements(model, rdata)
    z = {n: float(rdata.oMf[fid[n]].translation[2]) for n in FEET}
    p3 = {n: rdata.oMf[fid[n]].translation.copy() for n in FEET}
    com = pinocchio.centerOfMass(model, rdata, q).copy()
    return z, p3, com


def analyse(csv_path, verbose=True, dynamic=False):
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float64)
    N = len(data)
    Q = np.empty((N, model.nq))
    for t in range(N):
        q = data[t, :model.nq].copy()
        q[3:7] /= np.linalg.norm(q[3:7])   # renormalise quaternion
        Q[t] = q

    # per-frame kinematics + centroidal momentum
    zf, p3f, comf = [], [], []
    Lang = np.zeros((N, 3))                 # centroidal angular momentum
    for t in range(N):
        z, p3, com = kinematics(Q[t])
        zf.append(z); p3f.append(p3); comf.append(com)
    comf = np.array(comf)
    a_com = np.zeros((N, 3))
    dL = np.zeros((N, 3))
    if dynamic:
        # centroidal angular momentum per frame (manifold central-diff velocity)
        for t in range(N):
            tm, tp = max(0, t - 1), min(N - 1, t + 1)
            span = (tp - tm) * DT
            v = (pinocchio.difference(model, Q[tm], Q[tp]) / span
                 if span > 0 else np.zeros(model.nv))
            Lang[t] = pinocchio.computeCentroidalMomentum(model, rdata, Q[t], v).angular
        # Savitzky-Golay derivatives: raw 2nd/1st differences are noise-dominated
        # at 0.05 m/s, so smooth over ~0.3 s before differentiating.
        win = min(31, N if N % 2 else N - 1)
        if win >= 5:
            a_com = savgol_filter(comf, win, 3, deriv=2, delta=DT, axis=0)
            dL = savgol_filter(Lang, win, 3, deriv=1, delta=DT, axis=0)
    # quasi-static default: a_com = dL = 0 (near-exact at 0.05 m/s; ZMP == CoM)

    # classify support set per frame (robust to whole-body drift)
    z_series = {n: np.array([zf[t][n] for t in range(N)]) for n in FEET}
    support_sets = detect_support(z_series)

    # per-frame judge (transition frames with <3 supports get no polygon/LP test)
    frame = []
    for t in range(N):
        support = support_sets[t]
        c = comf[t]
        if len(support) < 3:
            frame.append(dict(support=support, margin=float("nan"),
                              zmp_margin=float("nan"), cone_viol=None,
                              vloads=np.array([]), transition=True))
            continue
        sp3 = [p3f[t][n] for n in support]
        sp2 = [p[:2] for p in sp3]
        sm = signed_margin(c[:2], sp2)
        Rnet = M * a_com[t] - M * np.array([0, 0, -G])          # = M a + [0,0,Mg]
        z_lp, forces = lp_cone_violation(sp3, c, Rnet, dL[t])
        fz = vertical_loads(sp3, c, Rnet[2], dL[t][0], dL[t][1])
        z_ground = min(p[2] for p in sp3)
        zmp = zmp_point(c, a_com[t], dL[t], z_ground)
        zm = signed_margin(zmp, sp2)
        frame.append(dict(support=support, margin=sm, zmp_margin=zm,
                          cone_viol=z_lp, vloads=fz, min_fz=float(fz.min()),
                          transition=False))

    # group into maximal same-support segments
    segs = []
    i = 0
    while i < N:
        j = i
        while j < N and support_sets[j] == support_sets[i]:
            j += 1
        segs.append((i, j))
        i = j

    # --- load split at neutral (answers the "front 32% / back 68%" concern) ---
    z0, p30, com0 = kinematics(Q[0])
    xf = np.mean([p30["FL_foot_link"][0], p30["FR_foot_link"][0]])
    xb = np.mean([p30["RL_foot_link"][0], p30["RR_foot_link"][0]])
    front = (com0[0] - xb) / (xf - xb)

    if verbose:
        print(f"\n=== {csv_path} : {N} rows, {len(segs)} support phases, "
              f"M={M:.3f} kg ({M*G:.1f} N) ===")
        print(f"load split @neutral: FRONT {front*100:.1f}%  BACK {(1-front)*100:.1f}%"
              f"  (each front {front/2*M*G:.1f} N, each hind {(1-front)/2*M*G:.1f} N)")
        print("%-8s %5s %-14s %8s %8s %7s %8s  %-26s %s" %
              ("phase", "frm", "support", "stat_m", "zmp_m", "LP?", "viol_N",
               "vert loads f_z [N] (neg=PULL)", "verdict"))

    worst = None
    n_fall = n_risky = 0
    for (a, b) in segs:
        support = support_sets[a]
        # <3 supports: a brief blip (<=TRANS_MAX_FRAMES) is just a swing switch and
        # is skipped; a LONGER 2-foot (or fewer) run is a genuine, statically
        # unstable phase -- a quadruped at rest cannot balance on <3 point feet
        # unless the CoM sits exactly on their line -- so it is judged a FALL.
        if len(support) < 3:
            sup_tag = "+".join(s.split("_")[0] for s in support) or "none"
            if (b - a) <= TRANS_MAX_FRAMES:
                if verbose:
                    print("%-8s %5d %-14s %8s %8s %7s %8s  %-26s %s" %
                          ("trans", b - a, sup_tag, "-", "-", "-", "-",
                           "(swing switch)", "skip"))
                continue
            n_fall += 1
            if verbose:
                print("%-8s %5d %-14s %8s %8s %7s %8s  %-26s %s" %
                      ("<3sup", b - a, sup_tag, "-", "-", "NO", "-",
                       "(only %d feet in support)" % len(support), "FALL"))
            key = (1, 1e9, 1.0)   # dominate the worst-phase ranking
            if worst is None or key > worst[0]:
                worst = (key, "<3sup", -1.0, None, "FALL", b - a,
                         frame[a], support, a)
            continue
        # aggregate: worst (smallest) static margin, worst (largest) cone viol
        k_ms = min(range(a, b), key=lambda k: frame[k]["margin"])
        k_lp = max(range(a, b), key=lambda k: (frame[k]["cone_viol"]
                                               if frame[k]["cone_viol"] is not None else -1e9))
        lifted = [n for n in FEET if n not in support]
        lift_tag = lifted[0].split("_")[0] if len(lifted) == 1 else \
            ("DS4" if len(lifted) == 0 else "+".join(l.split("_")[0] for l in lifted))
        ms = frame[k_ms]["margin"]
        zm = frame[k_lp]["zmp_margin"]
        viol = frame[k_lp]["cone_viol"]
        fz = frame[k_lp]["vloads"]
        feasible = viol is not None and viol <= CONE_TOL
        if not feasible:
            verdict = "FALL"; n_fall += 1
        elif ms < RISKY_MARGIN:
            verdict = "RISKY"; n_risky += 1
        else:
            verdict = "OK"
        sup_tag = "+".join(s.split("_")[0] for s in support)
        loads = " ".join(f"{s.split('_')[0]}:{v:+6.1f}" for s, v in zip(support, fz))
        if verbose:
            print("%-8s %5d %-14s %+8.4f %+8.4f %7s %8.2f  %-26s %s" %
                  (lift_tag, b - a, sup_tag, ms, zm,
                   "yes" if feasible else "NO",
                   viol if viol is not None else float("nan"), loads, verdict))
        # track the single most dangerous phase (largest cone violation, then
        # most-negative static margin)
        key = (0 if feasible else 1, viol if viol is not None else 0.0, -ms)
        if worst is None or key > worst[0]:
            worst = (key, lift_tag, ms, viol, verdict, b - a,
                     frame[k_ms], support, k_ms)

    # overall verdict
    overall = "FALL" if n_fall else ("RISKY" if n_risky else "PASS")
    if verbose:
        print(f"phases: FALL={n_fall}  RISKY={n_risky}  "
              f"OK={len(segs)-n_fall-n_risky}  =>  OVERALL: {overall}")
        # tip-over estimate on the worst phase (layer 4)
        if worst and worst[2] < 0 and len(worst[7]) >= 3:  # CoM outside a real triangle
            _, lift_tag, ms, viol, _, nfrm, fr, support, k = worst
            c = comf[k]
            sp2 = [p3f[k][n][:2] for n in support]
            d, q = nearest_edge_dist(c[:2], sp2)
            L3 = np.hypot(d, c[2] - min(p3f[k][n][2] for n in support))
            alpha = G * d / max(L3 ** 2, 1e-6)         # point-mass tip ang.accel
            T = nfrm * DT
            theta = 0.5 * alpha * T ** 2               # from rest over the phase
            print(f"tip-over @worst ({lift_tag}): CoM {d*100:.1f} cm OUTSIDE nearest "
                  f"support edge (already past the no-return line) -> "
                  f"ang.accel {alpha:.2f} rad/s^2, ~{np.degrees(theta):.1f} deg "
                  f"accumulated over the {T:.2f}s phase from rest.")
        if dynamic:
            print(f"dynamics (Savitzky-Golay): max|a_com|={np.abs(a_com).max():.3f} "
                  f"m/s^2, max|dL/dt|={np.abs(dL).max():.3f} N·m")
        else:
            print("mode: quasi-static (a_com=dL=0); pass --dynamic for inertial terms")
    return dict(path=csv_path, overall=overall, n_fall=n_fall, n_risky=n_risky,
                n_phases=len(segs), worst_viol=(worst[3] if worst else None),
                worst_margin=(worst[2] if worst else None))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Will-it-fall judge for a tripod CSV.")
    ap.add_argument("csv", nargs="*",
                    default=["trajectory_walking_sideways_sc_v0.05.csv"])
    ap.add_argument("--dynamic", action="store_true",
                    help="add CoM-accel & angular-momentum-rate terms "
                         "(Savitzky-Golay smoothed). Default: quasi-static "
                         "(a_com=dL=0), near-exact at 0.05 m/s and noise-free.")
    args = ap.parse_args()
    paths = args.csv
    results = [analyse(p, verbose=True, dynamic=args.dynamic) for p in paths]
    if len(results) > 1:
        print("\n===== SUMMARY =====")
        print("%-52s %-8s %6s %6s %8s %9s" %
              ("file", "verdict", "FALL", "RISKY", "worstN", "worst_m"))
        for r in results:
            wv = f"{r['worst_viol']:.1f}" if r["worst_viol"] is not None else "-"
            wm = f"{r['worst_margin']:+.3f}" if r["worst_margin"] is not None else "-"
            print("%-52s %-8s %6d %6d %8s %9s" %
                  (r["path"][-52:], r["overall"], r["n_fall"], r["n_risky"], wv, wm))
