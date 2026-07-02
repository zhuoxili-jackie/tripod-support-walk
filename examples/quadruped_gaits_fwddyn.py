import os
import signal
import sys
import time

import numpy as np
import pinocchio

import crocoddyl
from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem, plotSolution
from pcb_v2.pcbWrapper import pcb
WITHDISPLAY = "display" in sys.argv or "CROCODDYL_DISPLAY" in os.environ
WITHPLOT = "plot" in sys.argv or "CROCODDYL_PLOT" in os.environ
WITHODYN = "odyn" in sys.argv or "CROCODDYL_ODYN" in os.environ
signal.signal(signal.SIGINT, signal.SIG_DFL)

bars = list()
bars.append(np.array([0.4, 0.0, 0.3]))
bars.append(np.array([0.8, 0.0, 0.3]))
bars.append(np.array([1.2, 0.0, 0.3]))
bars.append(np.array([1.6, 0.0, 0.3]))


from pinocchio.robot_wrapper import RobotWrapper

_ROBOT_PATHS = {
    "go2": "/path/to/go2.urdf",
    # paths are relative to this examples/ directory (no leading slash!)
    "pcb_v2": os.path.join("pcb_v2", "pcb_v2", "urdf", "pcb_v2.urdf"),
}
dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(dir_path, _ROBOT_PATHS["pcb_v2"])
# directory that contains the "mesh/" subfolder referenced inside the URDF
mesh_dir = os.path.join(dir_path, "pcb_v2", "pcb_v2")
model_path = os.path.join(dir_path, "pcb_v2")
print(urdf_path)
print(model_path)

def load(robot_name):
    urdf = os.path.join(dir_path, _ROBOT_PATHS[robot_name])
    robot = RobotWrapper.BuildFromURDF(urdf, mesh_dir)
    return robot

robot = pcb()

robot_pcb = load("pcb_v2")
robot_pcb.model.referenceConfigurations["standing"] = robot.go_neutral()



q0 = robot_pcb.model.referenceConfigurations["standing"].copy()
v0 = pinocchio.utils.zero(robot_pcb .model.nv)
x0 = np.concatenate([q0, v0])
if crocoddyl.WITH_ODYN:
    x0_SQP = np.concatenate([q0, v0])

# Setting up the 3d walking problem
lfFoot, rfFoot, lhFoot, rhFoot = "FL_foot_link", "FR_foot_link", "RL_foot_link", "RR_foot_link"
gait = SimpleQuadrupedalGaitProblem(robot_pcb .model, lfFoot, rfFoot, lhFoot, rhFoot)




desired_velocity = np.array([0.0, -0.05, 0.0])  # [vx, vy, omega] 世界系，侧向行走 vy=0.1 m/s
import csv
import os

# 定义表头 - 使用实际关节名称
joint_names = [robot_pcb.model.names[i] for i in range(2, robot_pcb.model.njoints)]
fieldnames = ["root_pos_x", "root_pos_y", "root_pos_z", "root_rot_x", "root_rot_y", "root_rot_z", "root_rot_w"] + \
             joint_names + \
             ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]

output_path = "trajectory_trotting_acc_f005.csv"
# Setting up all tasks
GAITPHASES = []
for i in range(8):

    
    # 设定一个固定步态周期（以秒计）
    timeStep = 0.01
    stepKnots =35          # 摆动相节点数
    supportKnots = 5        # 支撑相节点数
    # 估算每一步的时长（针对 pacing 步态）
    # pacing 周期 ≈ 2 * (supportKnots * timeStep + stepKnots * timeStep)   // 简略估算
    T_step = 2 * ((supportKnots + stepKnots) * timeStep)

    # 计算步长以保证平均速度 = desired_velocity
    swing_length = np.linalg.norm(desired_velocity) * T_step   # 若方向已由 direction 决定，步长应为标量
    # 如果只想改变 y 方向的速度，则可直接用 desired_velocity[1] * T_step

    # 保持步高较小，避免摔倒
    swing_height = 0.1
    GAITPHASES.append({
    "trotting": {
    "stepLength": swing_length,
    "stepHeight": swing_height,
    "timeStep": timeStep,
    "stepKnots": stepKnots,
    "supportKnots": supportKnots,
    }
    })

solver = [None] * len(GAITPHASES)
if crocoddyl.WITH_ODYN:
    solverSQP = [None] * len(GAITPHASES)
for i, phase in enumerate(GAITPHASES):
    for key, value in phase.items():
        if key == "walking":
            # Creating a walking problem
            solver[i] = crocoddyl.SolverFDDP(
                gait.createWalkingProblem(
                    x0,
                    value["stepLength"],
                    value["stepHeight"],
                    value["timeStep"],
                    value["stepKnots"],
                    value["supportKnots"],
                    direction=(0.0, 1.0),   # 侧向行走
                )
            )
            if crocoddyl.WITH_ODYN:
                solverSQP[i] = crocoddyl.SolverOdynSQP(
                    gait.createWalkingProblem(
                        x0_SQP,
                        value["stepLength"],
                        value["stepHeight"],
                        value["timeStep"],
                        value["stepKnots"],
                        value["supportKnots"],
                        constraint=True,
                    )
                )
        elif key == "trotting":
            # Creating a trotting problem
            solver[i] = crocoddyl.SolverIntro(
                gait.createTrottingProblem(
                    x0,
                    value["stepLength"],
                    value["stepHeight"],
                    value["timeStep"],
                    value["stepKnots"],
                    value["supportKnots"],
                    direction=(0.0, np.sign(desired_velocity[1])),   # 侧向行走
                )
            )
            if crocoddyl.WITH_ODYN:
                print(x0_SQP)
                solverSQP[i] = crocoddyl.SolverOdynSQP(
                    gait.createTrottingProblem(
                        x0_SQP,
                        value["stepLength"],
                        value["stepHeight"],
                        value["timeStep"],
                        value["stepKnots"],
                        value["supportKnots"],
                        constraint=True,
                    )
                )
        elif key == "pacing":
            # Creating a pacing problem
            solver[i] = crocoddyl.SolverFDDP(
                gait.createPacingProblem(
                    x0,
                    value["stepLength"],
                    value["stepHeight"],
                    value["timeStep"],
                    value["stepKnots"],
                    value["supportKnots"],
                    direction=(0.0, 1.0),   # 侧向行走
                )
            )
            if crocoddyl.WITH_ODYN:
                solverSQP[i] = crocoddyl.SolverOdynSQP(
                    gait.createPacingProblem(
                        x0_SQP,
                        value["stepLength"],
                        value["stepHeight"],
                        value["timeStep"],
                        value["stepKnots"],
                        value["supportKnots"],
                        constraint=True,
                    )
                )
        elif key == "bounding":
            # Creating a bounding problem
            solver[i] = crocoddyl.SolverFDDP(
                gait.createBoundingProblem(
                    x0,
                    value["stepLength"],
                    value["stepHeight"],
                    value["timeStep"],
                    value["stepKnots"],
                    value["supportKnots"],
                )
            )
            if crocoddyl.WITH_ODYN:
                solverSQP[i] = crocoddyl.SolverOdynSQP(
                    gait.createBoundingProblem(
                        x0_SQP,
                        value["stepLength"],
                        value["stepHeight"],
                        value["timeStep"],
                        value["stepKnots"],
                        value["supportKnots"],
                        constraint=True,
                    )
                )
        elif key == "jumping":
            # Creating a jumping problem
            solver[i] = crocoddyl.SolverFDDP(
                gait.createJumpingProblem(
                    x0,
                    value["jumpHeight"],
                    value["jumpLength"],
                    value["timeStep"],
                    value["groundKnots"],
                    value["flyingKnots"],
                )
            )
            if crocoddyl.WITH_ODYN:
                solverSQP[i] = crocoddyl.SolverOdynSQP(
                    gait.createJumpingProblem(
                        x0_SQP,
                        value["jumpHeight"],
                        value["jumpLength"],
                        value["timeStep"],
                        value["groundKnots"],
                        value["flyingKnots"],
                        constraint=True,
                    )
                )

    # Added the callback functions
    if WITHPLOT:
        solver[i].setCallbacks(
            [
                crocoddyl.CallbackVerbose(),
                crocoddyl.CallbackLogger(),
            ]
        )
        if crocoddyl.WITH_ODYN:
            solverSQP[i].setCallbacks(
                [crocoddyl.CallbackVerbose(), crocoddyl.CallbackLogger()]
            )
    else:
        solver[i].setCallbacks([crocoddyl.CallbackVerbose()])
        if crocoddyl.WITH_ODYN:
            solverSQP[i].setCallbacks([crocoddyl.CallbackVerbose()])

    # Solving the problem with the OC solver
    xs = [x0] * (solver[i].problem.T + 1)
    us = solver[i].problem.quasiStatic([x0] * solver[i].problem.T)
    #print("*** SOLVE {key} (FeasShoot) ***".format_map(locals()))
    solver[i].setDynamicsSolver(crocoddyl.DynamicsSolverType.FeasShoot)
    solver[i].solve(xs, us, 100, False)
    #print("*** SOLVE {key} (MultiShoot) ***".format_map(locals()))
    solver[i].setDynamicsSolver(crocoddyl.DynamicsSolverType.MultiShoot)
    solver[i].solve(xs, us, 100, False)
    Ts = int(solver[i].problem.T / 3)
    #print("*** SOLVE {key} (HybridShoot: {Ts}) ***".format_map(locals()))
    solver[i].setDynamicsSolver(crocoddyl.DynamicsSolverType.HybridShoot, Ts)
    solver[i].solve(xs, us, 100, False)
    if crocoddyl.WITH_ODYN:
        xs = [x0_SQP] * (solverSQP[i].problem.T + 1)
        us = solverSQP[i].problem.quasiStatic([x0_SQP] * solverSQP[i].problem.T)
        #print("*** SOLVE {key} (OdynSQP) ***".format_map(locals()))
        solverSQP[i].solve(xs, us, 100, False)
    if i != len(GAITPHASES) - 1:
        print()

    # Defining the final state as initial one for the next phase
    x0 = solver[i].xs[-1]
    if crocoddyl.WITH_ODYN:
        x0_SQP = solverSQP[i].xs[-1]

print("Saving to:", os.path.abspath(output_path))   # 确认文件位置

with open(output_path, "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # 必须添加这个循环才能真正写入数据
    for i, solver_i in enumerate(solver):
        xs = solver_i.xs
        for t, x in enumerate(xs):
            q = x[:robot_pcb.model.nq]
            pinocchio.forwardKinematics(robot_pcb.model, robot_pcb.data, q)
            pinocchio.updateFramePlacements(robot_pcb.model, robot_pcb.data)
            base_frame_id = robot_pcb.model.getFrameId("Base_link")  # 改为实际名称
            base_pose = robot_pcb.data.oMf[base_frame_id]
            pos = base_pose.translation.copy()
            quat = pinocchio.SE3ToXYZQUAT(base_pose)[3:7]
            joint_pos = q[7:]

            row = {
                "root_pos_x": pos[0], "root_pos_y": pos[1], "root_pos_z": pos[2],
                "root_rot_x": quat[0], "root_rot_y": quat[1], "root_rot_z": quat[2], "root_rot_w": quat[3],
            }
            for j, val in enumerate(joint_pos):
                row[joint_names[j]] = val
           
            row["FL_foot_joint"] = 0.0
            row["FR_foot_joint"] = 0.0
            row["RL_foot_joint"] = 0.0
            row["RR_foot_joint"] = 0.0
            writer.writerow(row)

print("Done. Wrote", len(solver), "phases.")  
        
box_center = np.array([0.95, 0.0, 0.4])   # 盒子中心位置（放在地面上：高0.8，故中心z=0.4）
box_size = np.array([1, 4, 0.8])     # 长(x) 2，宽(y) 0.5，高(z) 0.8
# Display the entire motion
if WITHDISPLAY:
    import meshcat.geometry as g
    display = crocoddyl.MeshcatDisplay(robot_pcb )
    
    Mbox = pinocchio.SE3(np.eye(3), box_center)
    color = g.MeshLambertMaterial(
        color=display._rgbToHexColor([0.4, 0.4, 0.4, 1.0]),
        reflectivity=0.8,
    )
    display.robot.viewer["box"].set_object(g.Box(box_size), color)
    display.robot.viewer["box"].set_transform(Mbox.homogeneous)
            
    
    display.rate = -1
    display.freq = 1
    while True:
        for i, phase in enumerate(GAITPHASES):
            display.displayFromSolver(solver[i])
        if crocoddyl.WITH_ODYN:
            for i, phase in enumerate(GAITPHASES):
                display.displayFromSolver(solverSQP[i])
        time.sleep(1.0)


if WITHPLOT:
    plotSolution(solver, bounds=False, figIndex=1, show=False)
    for i, phase in enumerate(GAITPHASES):
        title = next(iter(phase.keys())) + " (phase " + str(i) + ")"
        log = solver[i].getCallbacks()[1]
        fig_idx = i + 3
        crocoddyl.plotConvergence(
            log.costs,
            log.pregs,
            log.dregs,
            log.grads,
            log.stops,
            log.steps,
            figTitle=title,
            figIndex=i + 3,
            show=False,
        )
    if crocoddyl.WITH_ODYN:
        plotSolution(solverSQP, figIndex=fig_idx + 1, show=False)
        for i, phase in enumerate(GAITPHASES):
            title = next(iter(phase.keys())) + " (phase " + str(i) + ")"
            log = solverSQP[i].getCallbacks()[1]
            crocoddyl.plotConvergence(
                log.costs,
                log.pregs,
                log.dregs,
                log.grads,
                log.stops,
                log.steps,
                figTitle=title,
                figIndex=i + fig_idx + 3,
                show=True if i == len(GAITPHASES) - 1 else False,
            )