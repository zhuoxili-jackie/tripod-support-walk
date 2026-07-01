import os
import signal
import sys
import time

import example_robot_data
import numpy as np
import pinocchio
from pcb.pcbWrapper import pcb
import crocoddyl
from crocoddyl.utils.quadruped import SimpleQuadrupedalGaitProblem, plotSolution

WITHDISPLAY = "display" in sys.argv or "CROCODDYL_DISPLAY" in os.environ
WITHPLOT = "plot" in sys.argv or "CROCODDYL_PLOT" in os.environ
signal.signal(signal.SIGINT, signal.SIG_DFL)


from pinocchio.robot_wrapper import RobotWrapper

_ROBOT_PATHS = {
    "go2": "/path/to/go2.urdf",
    "pcb": "/pcb/pcb/pcb.urdf",
}
dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(dir_path, "/pcb/urdf/pcb.urdf")
mesh_path = os.path.join(dir_path, "/pcb/meshes")
model_path = os.path.join(dir_path, "pcb")
print(urdf_path)
print(model_path)

def load(robot_name):
    urdf = _ROBOT_PATHS[robot_name]
    mesh_dir = os.path.dirname(model_path+mesh_path )
    robot = RobotWrapper.BuildFromURDF(model_path+urdf_path, mesh_dir)
    return robot

robot = pcb()
# Loading the anymal model
robot_pcb = load("pcb")
robot_model = robot.model
robot_pcb.model.referenceConfigurations["standing"] = robot.go_neutral()
lims = robot_pcb.model.effortLimit
print(lims)
lims *= 0.5  # reduced artificially the torque limits
robot_pcb.model.effortLimit = lims

# # Setting up the 3d walking problem
lfFoot, rfFoot, lhFoot, rhFoot = "FL_foot", "FR_foot", "RL_foot", "RR_foot"
gait = SimpleQuadrupedalGaitProblem(robot_pcb.model, lfFoot, rfFoot, lhFoot, rhFoot)

# # Defining the initial state of the robot
q0 = robot.go_neutral()
v0 = pinocchio.utils.zero(robot_pcb.model.nv)
x0 = np.concatenate([q0, v0])

# # Defining the walking gait parameters
walking_gait = {
    "stepLength": 0.25,
    "stepHeight": 0.25,
    "timeStep": 1e-2,
    "stepKnots": 25,
    "supportKnots": 2,
}

# Setting up the control-limited DDP solver
solver = crocoddyl.SolverBoxFDDP(
    gait.createWalkingProblem(
        x0,
        walking_gait["stepLength"],
        walking_gait["stepHeight"],
        walking_gait["timeStep"],
        walking_gait["stepKnots"],
        walking_gait["supportKnots"],
        direction=(0.0, 1.0),   # 侧向行走
    )
)

# Add the callback functions
print("*** SOLVE ***")
if WITHPLOT:
    solver.setCallbacks(
        [
            crocoddyl.CallbackVerbose(),
            crocoddyl.CallbackLogger(),
        ]
    )
else:
    solver.setCallbacks([crocoddyl.CallbackVerbose()])

# Solve the DDP problem
xs = [x0] * (solver.problem.T + 1)
us = solver.problem.quasiStatic([x0] * solver.problem.T)
result = solver.solve(xs, us, 100, False)

# solver.xs = list(solver.xs)
# solver.us = list(solver.us)
# # Plotting the entire motion
# if WITHPLOT:
#     # Plot control vs limits
#     plotSolution(solver, bounds=True, figIndex=1, show=False)

#     # Plot convergence
#     log = solver.getCallbacks()[1]
#     crocoddyl.plotConvergence(
#         log.costs,
#         log.pregs,
#         log.dregs,
#         log.grads,
#         log.stops,
#         log.steps,
#         figIndex=3,
#         show=True,
#     )

# # # Display the entire motion
if WITHDISPLAY:
    try:
        import gepetto

        gepetto.corbaserver.Client()
        cameraTF = [2.0, 2.68, 0.84, 0.2, 0.62, 0.72, 0.22]
        display = crocoddyl.GepettoDisplay(robot_pcb, 4, 4, cameraTF)
    except Exception:
        display = crocoddyl.MeshcatDisplay(robot_pcb)
    display.rate = -1
    display.freq = 1
    while True:
        display.displayFromSolver(solver)
        time.sleep(1.0)
