from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import torch
import time
import logging

from isaacsim.cortex.framework.cortex_world import CortexWorld #type:ignore
from isaacsim.cortex.framework.robot import add_ur10_to_stage #type:ignore
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfWaitState #type:ignore
from isaacsim.cortex.framework.dfb import DfBasicContext #type:ignore


def vis_debug_callback(step_size, bot_collision_vis=False):
    if bot_collision_vis:
        robot.motion_policy.visualize_collision_spheres()


class SampleStateTester(DfState):
    def __init__(self):
        super().__init__()
        self.target_pos = np.array([0.7, 0.0, 0.5])
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
        print(f"starting end effector pose: {self.robot.arm.get_end_effector_pose()}\n")
        print(f"starting joint positions: {self.robot_art.get_joint_positions()}\n")
        print(f"setting target pos as: {self.target_pos}")     

    def step(self):
        self.robot.arm.send_end_effector(target_position=self.target_pos)
        print(f"Moving to target position: {self.target_pos}")
        return None

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


class CloseGripperState(DfState):
    @property
    def robot(self):
        return self.context.robot
    
    def enter(self):
        print("<close suction gripper>")
        if not self.robot.suction_gripper.is_closed():
            self.robot.suction_gripper.close()
        
    def step(self):
        return self

class OpenGripperState(DfState):
    @property
    def robot(self):
        return self.context.robot

    def enter(self):
        print("<open suction gripper>")
        if self.robot.suction_gripper.is_closed():
            self.robot.suction_gripper.open()
    
    def step(self):
        return None


world = CortexWorld()
robot = world.add_robot(
    add_ur10_to_stage(
        name='ur10',
        prim_path="/World/ur10_suction"
    )
)
world.scene.add_default_ground_plane()

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





