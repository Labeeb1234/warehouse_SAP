from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import logging
import omni
import os

from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState #type:ignore
from isaacsim.cortex.framework.dfb import DfBasicContext #type:ignore

def vis_debug_callback(step_size, bot_collision_vis=False):
    if bot_collision_vis:
        robot.motion_policy.visualize_collision_spheres()


class SingularityTesterState(DfState):
    def __init__(self):
        super().__init__()
        print("<null space test state>")
        self.target_pos = None
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
        self.entry_time = time.time()
        posture_config = self.get_posture_config() + np.random.randn(6)
        self.target_pos, _ = self.robot.arm.get_end_effector_pose()

        print(f"setting target pos as: {self.target_pos}")
        print(f"starting end effector pose: {self.robot.arm.get_end_effector_pose()}\n")
        print(f"going to joint positions: {posture_config}\n")
        
        self.robot.arm.send_end_effector(target_position=self.target_pos, posture_config=posture_config)    

    def step(self):
        if time.time()-self.entry_time < 2.0:
            print(f"current joint positions(rad): {self.robot_art.get_joint_positions()}")
            print(f"current joint positions(deg): {self.robot_art.get_joint_positions()*(180.0/np.pi)}\n")
            return self
        print(f"exit")
        return None
    


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

# --------------------- adding the behaviour/decider networks here ----------------------
decider_network = DfNetwork(
    DfStateMachineDecider(
        DfStateSequence(
            [
                SingularityTesterState(),
            ], 
        
        loop=True)),
    context=DfBasicContext(robot)
)
world.add_decider_network(decider_network)
# ---------------------------------------------------------------------------------------

# world.add_physics_callback("sample_callback", vis_debug_callback)
world.run(simulation_app)


simulation_app.close()
