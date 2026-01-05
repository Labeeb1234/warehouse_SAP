from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import logging
import os 
import omni
from typing import Optional

from isaacsim.core.prims import RigidPrim #type:ignore
from pxr import Usd #type:ignore

import isaacsim.cortex.framework.math_util as math_utils #type:ignore
from isaacsim.cortex.framework.motion_commander import ApproachParams #type:ignore
from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState #type:ignore
from isaacsim.cortex.framework.dfb import DfBasicContext, DfRobotApiContext, DfDiagnosticsMonitor #type:ignore


def vis_debug_callback(step_size, bot_collision_vis=False):
    if bot_collision_vis:
        robot.motion_policy.visualize_collision_spheres()


class SampleStateContext(DfRobotApiContext):
    def __init__(self, robot, obj: Optional[RigidPrim], p_thresh: float=0.005, r_thresh: float=1.0):
        super().__init__(robot)
        self.reset()

        # adding the logical state monitors for feedback update and lookup
        self.add_monitors([
            SampleStateContext.monitor_end_effector
        ])

        self.obj = obj
        self.p_thresh, self.r_thresh = p_thresh, r_thresh
        self.target_pos, self.target_q = None, None
        self.target_reached = False

    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset

    def reset(self):
        self.target_reached = False
    
    def monitor_end_effector(self):
        if self.obj is None:
            print("[Warning] monitor_end_effector: self.obj is None.")
            return
        
        # extracting eef TF
        eef_pose = self.robot.arm.get_fk_T() # 4x4
        eef_pos, eef_q = math_utils.T2pq(eef_pose)

        # extracting target pose and TF
        self.target_pos, self.target_q = self.obj.get_world_poses()
        self.target_pos, self.target_q = np.squeeze(self.target_pos), np.squeeze(self.target_q)
        target_T = math_utils.pq2T(self.target_pos, self.target_q)

        self.is_target_reached = math_utils.transforms_are_close(
            T1=eef_pose, T2=target_T, 
            p_thresh=self.p_thresh, 
            R_thresh=self.r_thresh
        )

        # print(f"current eff pos: {eef_pos}")
        # print(f"current target pose: {self.target_pos}\n")

    

class SampleStateTester(DfState):
    def __init__(self):
        super().__init__()
        self.entry_time = None
        print("<Sample Test State>")

    @property
    def robot(self):
        return self.context.robot
    
    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset
    
    @staticmethod
    def get_posture_config(self):
        return self.robot.arm.motion_policy.get_default_cspace_position_target()

    def enter(self):
        self.entry_time = time.time()
        print(f"[state entry time]: {self.entry_time}")
        print(f"starting end effector pose: {self.robot.arm.get_end_effector_pose()}\n")
        print(f"starting joint positions: {self.robot_art.get_joint_positions()}\n")
        print(f"starting object pose: {self.context.obj.get_world_poses()}")
        print(f"starting object positions: {self.context.target_pos}\n")

    def step(self):
        if self.context.is_target_reached:
            print(f"Target reached stopping state here")
            # testing suction gripper (test lines only)
            # self.robot.suction_gripper.close()
            return None
        self.robot.arm.send_end_effector(target_position=self.context.target_pos)
        return self






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

# only for existing prims in the world
obj_prim_path = '/World/Cardbox_B2'
cb_prim = stage.GetPrimAtPath(obj_prim_path)
if not cb_prim.IsValid():
    raise RuntimeError(f"{obj_prim_path} is not a valid prim path")

obj = RigidPrim(
    prim_paths_expr=obj_prim_path,
    name='cb_rigid_view'
)

# --------------------- adding the behaviour/decider networks here ----------------------
decider_network = DfNetwork(
    DfStateMachineDecider(
        DfStateSequence(
            [
                SampleStateTester(),
            ], 
        
        loop=True)),
    context=SampleStateContext(robot, obj)
)
world.add_decider_network(decider_network)
# ---------------------------------------------------------------------------------------

# world.add_physics_callback("sample_callback", vis_debug_callback)
world.run(simulation_app)


simulation_app.close()





