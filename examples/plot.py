#!/usr/bin/env python3
"""
CSV Motion Player for Topsun V260 (URDF)
Loads the robot URDF and animates the trajectory from a CSV file.

CSV columns (first row header):
  root_pos_x, root_pos_y, root_pos_z,
  root_rot_x, root_rot_y, root_rot_z, root_rot_w,
  FL_hip_joint, FL_thigh_joint, FL_calf_joint,
  FR_hip_joint, FR_thigh_joint, FR_calf_joint,
  RL_hip_joint, RL_thigh_joint, RL_calf_joint,
  RR_hip_joint, RR_thigh_joint, RR_calf_joint,
  FL_foot_joint, FR_foot_joint, RL_foot_joint, RR_foot_joint

python3 plot.py  
--urdf /home/user/crocoddyl/examples/pcb/pcb/urdf/pcb.urdf
 /home/user/crocoddyl/examples/trajectory_trotting_acc_01.csv
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import argparse

# Joint names in the URDF (must match exactly)
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint",
]


def load_motion_csv(csv_path):
    """Load motion data from CSV, return frames (T, 7+16) and fps (default 50)."""
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1, dtype=np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # Expected columns: 7 (pos+quat) + 16 (joints) = 23
    assert data.shape[1] == 7 + len(JOINT_NAMES), \
        f"CSV column count {data.shape[1]} != 23"
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file', help='Path to the motion CSV file')
    parser.add_argument('--urdf', default=None,
                        help='Path to the robot URDF file. If not provided, uses the embedded URDF path relative to this script.')
    parser.add_argument('--fps', type=float, default=50,
                        help='Playback frames per second (default: 50)')
    parser.add_argument('--loop', action='store_true', help='Loop the animation')
    args = parser.parse_args()

    # Locate URDF
    urdf_path = args.urdf
    if urdf_path is None:
        # Assumes the URDF is in the same directory as this script or adjust as needed
        import os
        urdf_path = os.path.join(os.path.dirname(__file__), "topsun_v260.urdf")

    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # Load robot at origin with fixed base? No, floating base.
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0],
                          useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE)

    # Get joint indices for the controlled joints (excluding fixed base joint)
    num_joints = p.getNumJoints(robot_id)
    joint_indices = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode('utf-8')
        if name in JOINT_NAMES:
            joint_indices[name] = info[0]  # joint index
    # Ensure all joints found
    missing = set(JOINT_NAMES) - set(joint_indices.keys())
    if missing:
        print(f"Warning: missing joints in URDF: {missing}")

    # Load motion data
    frames = load_motion_csv(args.csv_file)
    num_frames = frames.shape[0]
    dt = 1.0 / args.fps

    print(f"Loaded {num_frames} frames. Playing... (close window to exit)")

    frame_idx = 0
    last_time = time.time()
    paused = False

    while p.isConnected():
        # Handle keyboard events
        keys = p.getKeyboardEvents()
        # Space to pause
        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
            paused = not paused
        # R to reset to beginning
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            frame_idx = 0

        if not paused:
            # Get current frame data
            row = frames[frame_idx]
            root_pos = row[0:3]
            root_quat = row[3:7]  # x,y,z,w from CSV
            joint_angles = row[7:]

            # Set base position and orientation
            # PyBullet expects (x,y,z) and (x,y,z,w) quaternion
            p.resetBasePositionAndOrientation(robot_id, root_pos.tolist(),
                                              root_quat[[ 0, 1, 2,3]].tolist())  # to w,x,y,z

            # Set joint positions
            for name, angle in zip(JOINT_NAMES, joint_angles):
                if name in joint_indices:
                    p.resetJointState(robot_id, joint_indices[name], angle)

            # Step simulation to update visuals (without physics)
            p.stepSimulation()

            # Advance frame
            frame_idx = (frame_idx + 1) % num_frames
            if frame_idx == 0 and not args.loop:
                paused = True
                print("Reached end of motion. Press SPACE to replay or R to reset.")

            # Sleep to maintain fps
            elapsed = time.time() - last_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = time.time()
        else:
            # When paused, still step to keep GUI responsive
            p.stepSimulation()
            time.sleep(0.01)

    p.disconnect()


if __name__ == '__main__':
    main()