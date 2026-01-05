from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import logging
import os 
import omni
import carb # type:ignore
from typing import Optional


from isaacsim.core.prims import RigidPrim, XFormPrim #type:ignore
from pxr import Usd #type:ignore

from isaacsim.core.utils.rotations import euler_angles_to_quat #type:ignore
import isaacsim.cortex.framework.math_util as math_utils #type:ignore
from isaacsim.cortex.framework.motion_commander import ApproachParams, PosePq #type:ignore
from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState, DfSetLockState #type:ignore
from isaacsim.cortex.framework.dfb import DfRobotApiContext, DfDiagnosticsMonitor #type:ignore


# ----------------------------------- DIAGNOSTICS MONITOR FOR THE LINE (a monitoring tool, custom logs) ----------------------------
# ----------------------------------------------------------------------------------------------------------------------------------

class ReachToPoseContext(DfRobotApiContext):
    def __init__(self, robot, target: Optional[XFormPrim]): # add monitor_fn if required
        super().__init__(robot)
        # reset context before first cycle of the decider network (cycle if in loop)
        self.target_reached = True
        self.target = target # target pose prim
        self.target_p, self.target_q = np.array([[0.0, 0.0, 0.5]]), np.array([euler_angles_to_quat(euler_angles=np.array([0.0, 0.0, 0.0]), degrees=True, extrinsic=False)])
        self.arm_info = {}
        self.joint_info = {}

    def reset(self):
        # reset some params
        # set target pose on reset
        self.target.set_world_poses(positions=self.target_p, orientations=self.target_q)
        self.target_reached = False
    
    # callbacks here 
    def monitor_task(self):
        pass

# give target pose details into the context so that custom target pose values can be passed online into the behaviour framework
# give pose target in task-space (RMPFLow motion policy)
# give custom approach to target params
# for now just add a feature to dynamically change the target pose on randomization
# monitor sensor feedbacks from arm (joint pos, vel and efforts) # flag to check if reached target pose
# monitor eef pose (T or p, q)




# creating a state machine to pass target pose into the ReachToPose state downstream (all have fallbacks to-->little messy without BT_CPP)
class SendProcessedTarget(DfState):
    '''ask for target input if none give a default target pose at every DeciderNetwork loop'''
    def __init__(self):
        super().__init__()

    @property
    def target_T(self):
        target_t = math_utils.pq2T(p=self.context.target_p, q=np.squeeze(self.context.target_q))
        return target_t

    def enter(self):
        print("<Send Target>")
        print(f"[INFO]-[SendTargetState]: Sending Target")
        
    def step(self):
        return None

    def exit(self):
        if self.target_T is not None:
            print(f"[INFO]-[SendTargetState]: Received Target TF.....exiting state moving on!")


class ReachToPose(DfState):
    def __init__(self):
        super().__init__()

    @property
    def robot(self):
        return self.context.robot
    
    @property
    def target(self):
        return self.context.target

    def enter(self):
        print(f"<Reach To Pose>")
        initial_joint_config = np.array([1.57, -1.57, -1.57, -1.57,  1.57,  0])
        # just a side track on entry set posture to the arm
        # process the target_p and target_q after randomizing
        if np.random.random() > 0.5:
            self.target_p = self.context.target_p  # add random offsets
        else:
            self.target_p = -self.context.target_p
            

    def step(self):
        print(f"[INFO]: Reaching...")
        # self.robot.arm.send_end_effector(target_position=self.target_p)
        return None
    


# ---------------------------------------------------------------------------------------------------------------------


    
# ------------ loading a custom USD scene into the cortex world system ----------------
working_dir = os.path.join("/home/inlabust/labeeb/warehouse_sap")
usd_file_path = os.path.join(working_dir, 'isolated_cortex_test_env.usd')
usd_context = omni.usd.get_context()
usd_context.open_stage(usd_file_path)
stage = usd_context.get_stage()

if stage is None:
    raise RuntimeError(f"Failed to open USD stage at {usd_file_path}")

if not stage.GetDefaultPrim():
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    print("Default prim set to /World.")

physics_path = "/World/PhysicsScene"
# ----------------------------------- WORLD AND BOT SETUP --------------------------------------------------
world = CortexWorld(
    physics_dt=1/60,
    rendering_dt=1/60,
    stage_units_in_meters=1.0,
    physics_prim_path=physics_path
)
robot = world.add_robot(
    add_ur10_to_stage(
        name='ur10',
        prim_path="/World/ur10_suction"
    )
)

# adding a prim for target pose
# creating prim
target_prim_path = "/World/GoalPose"
target_prim = stage.GetPrimAtPath(target_prim_path)
if not target_prim.IsValid():
    target_prim = stage.DefinePrim(target_prim_path, "Xform")

target_pose = XFormPrim(
    prim_paths_expr=target_prim_path,
    name="target_pose_prim_view",
    positions=np.array([[1.0, 0.0, 1.0]]),
    orientations=np.array([[1.0, 0.0, 0.0, 0.0]]) # w,x,y,z
)
print(f"GoalPose XformPrim created at: {target_pose.prim_paths}")


# --------------------- adding the behaviour/decider networks here ----------------------
decider_network = DfNetwork(
    DfStateMachineDecider(
        DfStateSequence(
            [
                SendProcessedTarget(),
                DfWaitState(wait_time=10.0),
                ReachToPose()
            ], 
        
        loop=True)),
    context=ReachToPoseContext(robot, target_pose)
)
world.add_decider_network(decider_network)
# ---------------------------------------------------------------------------------------

# world.add_physics_callback("sample_callback", vis_debug_callback)
world.run(simulation_app)


simulation_app.close()
