#!/usr/bin/env python3
"""Headless playback of a 23-column crocoddyl motion CSV.

Renders frames without a GUI / X server by using PyBullet's DIRECT mode +
TinyRenderer (CPU). Same joint mapping and CSV schema as ``plot.py``, so it
works on the exact files produced by ``quadruped_gaits_fwddyn.py``.

Examples
--------
# 6 key frames as PNGs:
python render_headless.py trajectory_trotting_acc_f005.csv \
    --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --outdir frames --nframes 6

# a GIF (needs Pillow, present in the croco310 env):
python render_headless.py trajectory_trotting_acc_f005.csv \
    --urdf pcb_v2/pcb_v2/urdf/pcb_v2.urdf --outdir frames --nframes 60 --gif motion.gif

Requirements: pybullet + numpy (+ Pillow for --gif / PNG). If Pillow is
missing, PNGs are written as raw .ppm instead.
"""
import argparse
import os

import numpy as np
import pybullet as p
import pybullet_data

# Joint names in URDF order matching CSV columns 8..23 (must match the URDF).
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint",
]


def load_motion_csv(csv_path):
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    assert data.shape[1] == 7 + len(JOINT_NAMES), \
        f"CSV column count {data.shape[1]} != {7 + len(JOINT_NAMES)}"
    return data


def save_rgb(rgb, path):
    """Write an (H, W, 3) uint8 array as PNG (Pillow) or .ppm fallback."""
    try:
        from PIL import Image
        Image.fromarray(rgb).save(path)
        return path
    except Exception:
        path = os.path.splitext(path)[0] + ".ppm"
        h, w = rgb.shape[:2]
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (w, h))
            f.write(rgb.tobytes())
        return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_file", help="path to the 23-column motion CSV")
    ap.add_argument("--urdf", required=True, help="path to the robot URDF")
    ap.add_argument("--outdir", default="frames", help="output directory for frames")
    ap.add_argument("--nframes", type=int, default=6, help="number of frames to render")
    ap.add_argument("--gif", default=None, help="also assemble frames into this GIF path")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    data = load_motion_csv(args.csv_file)
    T = data.shape[0]
    print(f"CSV: {args.csv_file}  shape={data.shape}  frames={T}")

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(args.urdf, basePosition=[0, 0, 0], useFixedBase=False,
                       flags=p.URDF_USE_INERTIA_FROM_FILE)

    idx = {}
    for j in range(p.getNumJoints(robot)):
        name = p.getJointInfo(robot, j)[1].decode()
        if name in JOINT_NAMES:
            idx[name] = j
    missing = set(JOINT_NAMES) - set(idx)
    print(f"joints matched: {len(idx)}/{len(JOINT_NAMES)} | missing: {missing or 'none'}")

    proj = p.computeProjectionMatrixFOV(
        fov=55, aspect=args.width / args.height, nearVal=0.05, farVal=20)

    def render(row, path):
        pos = row[0:3].tolist()
        quat = row[3:7].tolist()  # xyzw (pybullet order)
        p.resetBasePositionAndOrientation(robot, pos, quat)
        for name, ang in zip(JOINT_NAMES, row[7:]):
            if name in idx:
                p.resetJointState(robot, idx[name], float(ang))
        p.stepSimulation()
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[pos[0], pos[1], pos[2] - 0.1],
            distance=1.6, yaw=50, pitch=-25, roll=0, upAxisIndex=2)
        _, _, rgb, _, _ = p.getCameraImage(
            args.width, args.height, view, proj, renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(np.array(rgb, dtype=np.uint8),
                         (args.height, args.width, 4))[:, :, :3]
        return save_rgb(rgb, path)

    sel = np.linspace(0, T - 1, args.nframes).astype(int)
    print("rendering frame indices:", sel.tolist())
    paths = []
    for k, fi in enumerate(sel):
        out = os.path.join(args.outdir, f"frame_{k:03d}_idx{fi:04d}.png")
        paths.append(render(data[fi], out))
        r = data[fi]
        print(f"  frame {k:3d}: idx={fi:5d} base=({r[0]:+.3f},{r[1]:+.3f},{r[2]:+.3f})")

    print("base X: %.3f..%.3f  Y: %.3f..%.3f  Z: %.3f..%.3f" % (
        data[:, 0].min(), data[:, 0].max(), data[:, 1].min(), data[:, 1].max(),
        data[:, 2].min(), data[:, 2].max()))

    if args.gif:
        try:
            from PIL import Image
            imgs = [Image.open(f).convert("RGB") for f in paths]
            imgs[0].save(args.gif, save_all=True, append_images=imgs[1:],
                         duration=60, loop=0)
            print("GIF ->", args.gif, os.path.getsize(args.gif), "bytes")
        except Exception as e:
            print("GIF skipped (needs Pillow):", e)

    p.disconnect()
    print("done ->", args.outdir)


if __name__ == "__main__":
    main()
