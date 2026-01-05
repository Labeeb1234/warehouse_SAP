from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import os
# import threading
import omni
from typing import Optional, Any

from isaacsim.core.prims import RigidPrim #type:ignore
from pxr import Usd #type:ignore

from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.core.api.objects import VisualCuboid #type:ignore
import isaacsim.cortex.framework.math_util as math_utils #type:ignore

from isaacsim.cortex.framework.motion_commander import ApproachParams, PosePq, MotionCommand #type:ignore
from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState #type:ignore
from isaacsim.cortex.framework.dfb import DfBasicContext, DfRobotApiContext, DfDiagnosticsMonitor #type:ignore



# ------------------------------------------- Feedback and Diagnostics Monitor --------------------------------
class PickAndPlaceDiagnosticsMonitor(DfDiagnosticsMonitor):
    def __init__(self, print_dt: int = 1.0, diagnostic_fn: Any = None):
        super().__init__(print_dt)
        self.diagnostic_fn = diagnostic_fn

    def diagnostics_logger(self, context):
        print(context.eef_info) # need to fix this display format not good
        print(context.obj_info)

class PickAndPlaceContext(DfRobotApiContext):
    def __init__(self, robot, obj: Optional[RigidPrim], p_thresh: float = 0.005, r_thresh: float = 1.0):
        super().__init__(robot)
        self.reset()
        self.add_monitors([
            PickAndPlaceContext.object_monitor,
            PickAndPlaceContext.eef_monitor
        ])

        # gripper params
        self.obj_attached = False
        # prims and external prim params
        self.obj = obj # object params

        # target params
        self.p_thresh, self.r_thresh = p_thresh, r_thresh
        self.go_home = False # home target params
        self.home_target = None # this going to be a Posepq data type
        self.is_target_pose_reached = False
        self.target_pos, self.target_q = None, None # eef desired target pose vectors (p,q) --> MotionCommand format
    
    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset

    def reset(self):
        self.is_target_pose_reached = False
        self.obj_attached = False
        self.go_home = False

    def eef_monitor(self):
        # extracting eef info (sim sensor feedback data)
        eef_pose_T = self.robot.arm.get_fk_T() # eef pose TF
        eef_pos, eef_q = math_utils.T2pq(eef_pose_T)
        
        if self.target_pos is not None and self.target_q is not None and not self.go_home:
            # reach handling logic here
            target_pose_pq = PosePq(self.target_pos, self.target_q)
            target_pose_T = target_pose_pq.to_T()
            self.is_target_pose_reached = math_utils.transforms_are_close(
                T1=eef_pose_T, T2=target_pose_T,
                p_thresh=self.p_thresh,
                R_thresh=self.r_thresh
            )
            eef_info = {
                "eef_pose": PosePq(eef_pos, eef_q), # placeholder
                "arm_joint_pos": self.robot_art.get_joint_positions(), # 6 joints since 6DOF
                "eef_dist_to_target": np.linalg.norm(np.array([self.target_pos[:3]-eef_pos[:3]])),
                # "eef_orient_to_target": float # relative quaternion (all calculations in quaternions since they are computationally faster)
                # extra optional params if required like joint vel, efforts, etc...
            }
        elif self.go_home:
            home_config = self.robot.arm.motion_policy.get_default_cspace_position_target()
            self.home_target = self.robot.arm.get_fk_pq(config=home_config)
            home_target_T = self.home_target.to_T()
            self.is_target_pose_reached = math_utils.transforms_are_close(
                T1=eef_pose_T, T2=home_target_T, 
                p_thresh=self.p_thresh,
                R_thresh=self.r_thresh
            )
            eef_info = {
                "eef_pose": PosePq(eef_pos, eef_q), # placeholder
                "arm_joint_pos": self.robot_art.get_joint_positions(), # 6 joints since 6DOF
                "eef_dist_to_target": np.linalg.norm(np.array([self.home_target.p[:3]-eef_pos[:3]])),
                # "eef_orient_to_target": float # relative quaternion (all calculations in quaternions since they are computationally faster)
                # extra optional params if required like joint vel, efforts, etc...
            }
        else:
            print("No Target Pose available ")
            eef_info = {
                "eef_pose": PosePq(eef_pos, eef_q), # placeholder
                "arm_joint_pos": self.robot_art.get_joint_positions(), # 6 joints since 6DOF
                "eef_dist_to_target": [],
                # "eef_orient_to_target": float # relative quaternion (all calculations in quaternions since they are computationally faster)
                # extra optional params if required like joint vel, efforts, etc...
            }

    def object_monitor(self):
        obj_pos, obj_q = self.obj.get_world_poses()
        self.target_pos, self.target_q = np.squeeze(obj_pos), np.squeeze(obj_q)
        obj_info = {
            "obj_pose": PosePq(self.target_pos, self.target_q),
            "num_obj_pickloc": 1,
            "num_obj_placeloc": 0
        }

#--------------------------------------------------------------------------------------------------------------



# -------------------------------------- STATE MACHINES -------------------------------------------------------
class ReachToTarget(DfState):
    def __init__(self, task_msg: str, approach_params: Optional[np.ndarray], posture_config: Optional[np.ndarray]):
        super().__init__()
        print(f"<Reach To {task_msg}>")
        self.__entry_time = None
        self.__task_msg = task_msg

    def enter(self):
        print(f"\nGoing To Target Pose: ({self.context.target_pos}, {self.context.target_q})")

    def step(self):
        if self.context.is_target_pose_reached:
            return None

        self.context.robot.arm.send_end_effector(target_position=self.context.target_pos)
        return self

    def exit(self):
        print(f"Target Pose Reached!\n")

class GoToHome(DfState):
    def __init__(self):
        super().__init__()
        print("<Going Home>")
        self.__entry_time = None

    def enter(self):
        home_config = self.context.robot.arm.motion_policy.get_default_cspace_position_target()
        self.context.home_target = self.context.robot.arm.get_fk_pq(config=home_config)
        print(f"Home Pose: ({self.context.home_target.p}, {self.context.home_target.q})")
        self.context.go_home = True

    def step(self):
        if self.context.is_target_pose_reached:
            self.context.go_home = False
            return None
        
        self.context.robot.arm.send_end_effector(target_position=self.context.home_target.p)
        return self
    
    def exit(self):
        print("Reached home pose\n")

class ReachToPick(ReachToTarget):
    def __init__(
        self, 
        approach_params=ApproachParams(direction=0.3*np.array([0.0, 0.0, -1.0]), std_dev=0.005), 
        posture_config=None
    ):
        super().__init__("Reach To Pick", approach_params, posture_config)

class SuctionOpen(DfState):
    def __init__(self):
        print("<Suction Gripper Off>")
        self.__entry_time = None
    
    def enter(self):
        # immediate open ig
        self.context.robot.suction_gripper.open()

    def step(self):
        # if still closed try reopening repeatedly
        if self.context.robot.suction_gripper.is_closed():
            return self
        else:
            return None

    def exit(self):
        print("Suction gripper opened\n")

class SuctionClose(DfState):
    def __init__(self):
        print("<Suction Gripper On>")
        self.__entry_time = None
    
    def enter(self):
        self.context.robot.suction_gripper.close()

    def step(self):
        if not self.context.robot.suction_gripper.is_closed():
            return self
        else:
            return None

    def exit(self):
        print("Suction gripper closed\n")

# I think these should be eventually converted to Decider Nodes for more reponsive task planning design
class PickUpObject(DfState):
    pass

class PlaceObject(DfState):
    pass

# -------------------------------------------------------------------------------------------------------------





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
                ReachToPick(),
                SuctionClose(),
                GoToHome(),
            ], 
        
        loop=False)),
    context=PickAndPlaceContext(robot, obj)
)
world.add_decider_network(decider_network)
# ---------------------------------------------------------------------------------------

# world.add_physics_callback("sample_callback", vis_debug_callback)
world.run(simulation_app)


simulation_app.close()
