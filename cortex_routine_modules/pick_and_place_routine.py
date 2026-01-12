from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import os
# import threading
import omni
from typing import Optional, Any

from isaacsim.core.prims import RigidPrim, XFormPrim #type:ignore
from pxr import Usd #type:ignore

from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.core.api.objects import VisualCuboid #type:ignore
import isaacsim.cortex.framework.math_util as math_utils #type:ignore

from isaacsim.cortex.framework.motion_commander import ApproachParams, PosePq, MotionCommand #type:ignore
from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfDecider, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState #type:ignore
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
    def __init__(self, robot, obj: Optional[RigidPrim], place_loc: Optional[RigidPrim], p_thresh: float = 0.005, r_thresh: float = 1.0):
        super().__init__(robot)
        self.reset()
        self.add_monitors([
            PickAndPlaceContext.target_monitor,
            PickAndPlaceContext.eef_monitor
        ])

        # gripper params
        self.obj_attached = False
        # prims and external prim params
        self.obj = obj # object params
        self.place_loc = place_loc # place location prim

        # target params
        self.p_thresh, self.r_thresh = p_thresh, r_thresh
        self.go_home, self.is_home = False, False # home target params
        self.is_target_pose_reached = False
        self.target_pos, self.target_q = None, None # eef desired target pose vectors (p,q) --> MotionCommand format
    
    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset

    def reset(self):
        self.is_target_pose_reached = False
        self.obj_attached = False
        self.go_home = False
        self.is_home = False

    def eef_monitor(self):
        # extracting eef info (sim sensor feedback data)
        eef_pose_T = self.robot.arm.get_fk_T() # eef pose TF
        eef_pos, eef_q = math_utils.T2pq(eef_pose_T)
        
        msg = f"""
            Target Reach Flag: {self.is_target_pose_reached},
            Go Home State Flag: {self.go_home},
            Object Attached Flag: {self.obj_attached},
            Is Home Flag: {self.is_home},
            Joint Config: {self.robot_art.get_joint_positions()}
        """
        print(f"Monitor:\n{msg}\n")

        if self.target_pos is not None and self.target_q is not None:
            if not self.go_home:
                # reach handling logic here
                target_pose_pq = PosePq(self.target_pos, self.target_q)
                target_pose_T = target_pose_pq.to_T()
                self.is_target_pose_reached = math_utils.transforms_are_close(
                    T1=eef_pose_T, T2=target_pose_T,
                    p_thresh=self.p_thresh,
                    R_thresh=self.r_thresh
                )
            else:
                target_pose_pq = PosePq(self.target_pos, self.target_q)
                home_target_T = target_pose_pq.to_T()
                self.is_target_pose_reached = math_utils.transforms_are_close(
                    T1=eef_pose_T, T2=home_target_T, 
                    p_thresh=self.p_thresh,
                    R_thresh=self.r_thresh
                )
        else:
            print("No Target Pose available")

    def target_monitor(self):
        # only if the arm has an object and the arm is at home pose set the target location as place location
        if not self.obj_attached and not self.go_home:
            obj_pos, obj_q = self.obj.get_world_poses()
            self.target_pos, self.target_q = np.squeeze(obj_pos), np.squeeze(obj_q)
        elif self.is_home and self.obj_attached:
            place_loc_pos, place_loc_q = self.place_loc.get_world_poses()
            self.target_pos, self.target_q = np.squeeze(place_loc_pos), np.squeeze(place_loc_q)
        elif self.go_home:
            home_config = self.robot.arm.motion_policy.get_default_cspace_position_target()
            home_target = self.robot.arm.get_fk_pq(config=home_config)
            self.target_pos, self.target_q = home_target.p, home_target.q




#--------------------------------------------------------------------------------------------------------------



# -------------------------------------- STATE MACHINES -------------------------------------------------------
class ReachToTarget(DfState):
    def __init__(self, task_msg: str, approach_params: Optional[np.ndarray], posture_config: Optional[np.ndarray]):
        super().__init__()
        print(f"<Reach To {task_msg}>") # goes to any target except home state
        self.__entry_time = None
        self.__task_msg = task_msg
        self.__approach_params = approach_params
        self.__posture_config = posture_config

    def enter(self):
        if self.context.is_home:
            self.context.is_home = False
        print(f"\nGoing To Target Pose: ({self.context.target_pos}, {self.context.target_q})")

    def step(self):
        if self.context.is_target_pose_reached:
            return None

        if self.__approach_params is not None and self.__posture_config is not None:
            self.context.robot.arm.send_end_effector(
                target_position=self.context.target_pos,
                approach_params=self.__approach_params,
                posture_config=self.__posture_config
            )
        else:
            self.context.robot.arm.send_end_effector(
                target_position=self.context.target_pos
            )
        return self

    def exit(self):
        print(f"Target Pose Reached!\n")

class GoToHome(DfState):
    def __init__(self):
        super().__init__()
        print("<Going Home>")
        self.__entry_time = None
        self.__posture_config = None

    def enter(self):
        self.context.go_home = True
        self.__posture_config = self.context.robot.arm.motion_policy.get_default_cspace_position_target()
        print(f"Home Pose: ({self.context.target_pos}, {self.context.target_q})")
        
    def step(self):
        if self.context.is_target_pose_reached:
            self.context.go_home = False
            self.context.is_home = True
            return None
        
        self.context.robot.arm.send_end_effector(
            target_position=self.context.target_pos,
            posture_config=self.__posture_config
        )
        return self
    
    def exit(self):
        print("Reached home pose\n")

class ReachToPick(ReachToTarget):
    def __init__(
        self, 
        approach_params=ApproachParams(direction=0.3*np.array([0.0, 0.0, -1.0]), std_dev=0.005), 
        posture_config=np.array([-1.8462447, -2.4538538, -1.1502568, -1.1467233, 1.57, 0.0])
    ):
        super().__init__("Reach To Pick", approach_params, posture_config)

class ReachToPlace(ReachToTarget):
    def __init__(self, 
        approach_params=ApproachParams(direction=0.3*np.array([0.0, 0.0, -1.0]), std_dev=0.005), 
        posture_config=np.array([-9.9106693e-01, -2.0576217, -1.7371155, -0.78, 1.7451841, 0.0])
    ):
        super().__init__("Reach To Place", approach_params, posture_config)



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
            self.context.robot.suction_gripper.open()
            return self
        else:
            if self.context.obj_attached:
                self.context.obj_attached = False
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
            self.context.robot.suction_gripper.close()
            return self
        else:
            self.context.obj_attached = True
            return None

    def exit(self):
        print("Suction gripper closed\n")

# I think these should be eventually converted to Decider Nodes for more reponsive task planning design
class PickUpObject(DfDecider):
    pass

class PlaceObject(DfDecider):
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

# creating a prim for the place location (non-existent the default USD world file)
place_loc_prim_path = '/World/place_loc'
place_loc_prim = stage.GetPrimAtPath(place_loc_prim_path)
if not place_loc_prim.IsValid():
    raise RuntimeError(f"{place_loc_prim} is not a valid prim path")

place_loc = XFormPrim(
    prim_paths_expr=place_loc_prim_path,
    name='place_loc_view'
)


# --------------------- adding the behaviour/decider networks here ----------------------
decider_network = DfNetwork(
    DfStateMachineDecider(
        DfStateSequence(
            [
                ReachToPick(),
                SuctionClose(),
                DfWaitState(wait_time=0.5),
                GoToHome(),
                DfWaitState(wait_time=0.5),
                ReachToPlace(),
                SuctionOpen(),
                DfWaitState(wait_time=0.5),
                GoToHome()
            ], 
        
        loop=False)),
    context=PickAndPlaceContext(robot, obj, place_loc)
)
world.add_decider_network(decider_network)
# ---------------------------------------------------------------------------------------

# world.add_physics_callback("sample_callback", vis_debug_callback)
world.run(simulation_app)


simulation_app.close()
