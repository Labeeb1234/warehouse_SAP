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

from isaacsim.core.api.objects import VisualCuboid #type:ignore
import isaacsim.cortex.framework.math_util as math_utils #type:ignore
from isaacsim.cortex.framework.motion_commander import ApproachParams, PosePq #type:ignore
from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState, DfSetLockState #type:ignore
from isaacsim.cortex.framework.dfb import DfBasicContext, DfRobotApiContext, DfDiagnosticsMonitor #type:ignore

# ----------------------------------- DIAGNOSTICS MONITOR FOR THE LINE ------------------------------------
class PickPlaceDiagnosticsMonitor(DfDiagnosticsMonitor):
    def __init__(self, print_dt=1.0, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn

    def print_diagnostics(self, context):
        # print(context.joint_info)
        print(context.eef_info)

# ---------------------------------------------------------------------------------------------------------
class PickPlaceContext(DfRobotApiContext):
    def __init__(
        self, 
        robot, 
        obj: Optional[RigidPrim], 
        p_thresh: float=0.003, 
        r_thresh: float=1.0,  
        monitor_fn=None
    ):
        super().__init__(robot)
        self.reset()


        self.home = False # temp flag

        self.obj = obj
        self.p_thresh, self.r_thresh = p_thresh, r_thresh
        self.target_pos, self.target_q = None, None        

        self.pp_diagnostics_monitor = PickPlaceDiagnosticsMonitor(print_dt=0.1, diagnostic_fn=monitor_fn)

        self.eef_info = None
        self.joint_info = None

        self.add_monitors(
            [PickPlaceContext.monitor_end_effector,
             PickPlaceContext.monitor_joint_info,
             self.pp_diagnostics_monitor.monitor
            ]
        )

    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset

    def reset(self):
        self.is_target_reached = False
        self.is_grasped = False
    
    def monitor_end_effector(self):
        if self.obj is None:
            print("[Warning] monitor_end_effector: self.obj is None.")
            return
        
        eef_pose = self.robot.arm.get_fk_T() # 4x4
        eef_pos, eef_q = math_utils.T2pq(eef_pose)
        self.target_pos, self.target_q = self.obj.get_world_poses()
        self.target_pos, self.target_q = np.squeeze(self.target_pos), np.squeeze(self.target_q)
        target_T = math_utils.pq2T(self.target_pos, self.target_q)
    
        errT = eef_pose-target_T
        errR, errP = math_utils.unpack_T(errT)
        nerrR = np.linalg.norm(errR)

        self.is_target_reached = math_utils.transforms_are_close(
            T1=eef_pose, T2=target_T, p_thresh=self.p_thresh, R_thresh=self.r_thresh
        )
        # self.is_target_reached = np.linalg.norm(eef_pos-self.target_pos) < self.p_thresh

        self.eef_info = {
            "current eef pose (FK_T)": eef_pose, 
            "target pose": target_T,
            "distance to target": np.linalg.norm(errP),
            "angle to target(avg_along3)": nerrR,
            "target reached": self.is_target_reached,
            "grasped": self.is_grasped
        }
                
    def monitor_joint_info(self):
        joint_positions = self.robot_art.get_joint_positions()
        num_active_joints = self.robot.arm.num_controlled_joints
        self.joint_info = {
            "cspace_joints": num_active_joints,
            "joint_position(rad)": joint_positions,
            "joint_position(deg)": joint_positions*(180/np.pi),
        }
    
# ------------------------------------------------------------------------------------------------
class GoToHome(DfState):
    def __init__(self):
        super().__init__()
        self.home = False
        self.home_pose = None
    
    @property
    def robot(self):
        return self.context.robot

    def enter(self):
        print("<Go To Home>")
        initial_joint_config = np.array([-1.57, -1.57, -1.57, -1.57,  1.57,  0])
        # FK output 
        self.home_pose = self.robot.arm.get_fk_pq(config=initial_joint_config)
        self.robot.arm.send_end_effector(
            target_pose=self.home_pose, posture_config=initial_joint_config
        )

    def step(self):
        eef_p, _ = math_utils.T2pq(self.robot.arm.get_fk_T())
        home_p, _ = self.home_pose
        self.home = np.linalg.norm(home_p-eef_p) < 0.003 # 3mm accuracy
        if self.home:
            return None
        
        return self

class ReachToPick(DfState):
    def __init__(self):
        super().__init__()
        self.entry_time = None

    @property
    def robot(self):
        return self.context.robot
    
    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset
    
    def get_posture_config(self):
        return self.robot.arm.motion_policy.get_default_cspace_position_target()

    def enter(self):
        print("<Reach To Pick>")
        self.entry_time = time.time()
        print(f"setting target pos(p) and orientation(q) as: [{self.context.target_pos}--{self.context.target_q}]")
        print(f"initial end effector pose: {self.robot.arm.get_end_effector_pose()}\n")
        print(f"initial joint positions: {self.robot_art.get_joint_positions()}\n")
        
    def step(self):
        approach_params = ApproachParams(direction=0.3*np.array([0.0, 0.0, -1.0]), std_dev=0.005)
        # posture_config = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, -3.4733276e-08])
        posture_config = self.robot.default_config
        self.robot.arm.send_end_effector(
            target_position=self.context.target_pos, approach_params=approach_params, posture_config=posture_config
        )
        if self.context.is_target_reached:
            return None
        
        return self

class ReachToPlace(DfState):
    def __init__(self):
        super().__init__()
        self.entry_time = None

    @property
    def robot(self):
        return self.context.robot
    
    @property
    def robot_art(self):
        return self.robot.arm.articulation_subset
    
    def get_posture_config(self):
        return self.robot.arm.motion_policy.get_default_cspace_position_target()
    
    def enter(self):
        print("<Reach To Place>")
        # hardcoded placement position

# ------------------------------------- end effector tool states -------------------------------
class CloseSuctionGripperWithRetries(DfState):
    def enter(self):
        pass
        
    def step(self):
        gripper = self.robot.suction_gripper
        gripper.close()
        if gripper.is_close():
            return None
        return self
    
class CloseSuctionGripper(DfState):
    def enter(self):
        print("<close gripper>")
        self.context.robot.suction_gripper.close()
    def step(self):
        return None

class OpenSuctionGripper(DfState):
    def enter(self):
        print("<open gripper>")
        self.context.robot.suction_gripper.open()
    def step(self):
        return None
    
# ------------------------------------------- HELPER STATES ----------------------------------------
class DoNothing(DfState):
    def enter(self):
        self.context.robot.arm.clear()

    def step(self):
        print(self.context.robot.arm.target_prim.get_world_pose())
        return self

# --------------------------------------------------------------------------------------------------

# --------------- STATE MACHINE DECIDER NODES (future implementation; for better functionality) -----
# --------------------------------------------------------------------------------------------------



def main():

    def vis_debug_callback(step_size, bot_collision_vis=False):
        if bot_collision_vis:
            robot.motion_policy.visualize_collision_spheres()

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
                    # DfWaitState(wait_time=2.0),
                    # ReachToPlace(),
                    # DoNothing(),
                ], 
            
            loop=True)),
        context=PickPlaceContext(robot, obj)
    )
    world.add_decider_network(decider_network)

    # ------------------------------------ RUNNING SIMULATION APP ----------------------------
    world.run(simulation_app)
    simulation_app.close()

if __name__ == '__main__':
    main()





