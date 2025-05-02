import numpy as np
import os

import omni
import omni.graph.core as og # type: ignore
from isaacsim.core.prims import SingleArticulation as Articulation  # type: ignore
from isaacsim.core.prims import XFormPrim, RigidPrim  # type: ignore
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats  # type: ignore
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy  # type: ignore
from isaacsim.core.api.objects.cuboid import FixedCuboid  # type: ignore


# Global variables to persist across compute calls
arm_art = None
rmpflow = None
articulation_rmpflow = None
target = None
step = 1/60 # physics step size
dbg_mode = True

def setup(db: og.Database):
    global arm_art, rmpflow, articulation_rmpflow, target

    print(f"Setting up.....")

    home_dir = os.path.expanduser("~")
    ws_dir = home_dir + '/labeeb/warehouse_sap'
    
    # Load robot articulation
    robot_prim_path = "/World/ur10_short_suction"
    arm_art = Articulation(robot_prim_path)
    arm_art.initialize()
    
    # Load target transform
    target_prim_path = "/World/target"
    target = XFormPrim(target_prim_path)

    # RMP-Flow configuration
    mg_extension_path = home_dir + "/labeeb/isaac_sim_45/exts/isaacsim.robot_motion.motion_generation"
    rmpflow_config_dir = os.path.join(mg_extension_path, 'motion_policy_configs')
    rmpflow_config = rmpflow_config_dir + "/ur10/rmpflow/ur10_rmpflow_config.yaml"

    rmpflow = RmpFlow(
        robot_description_path=ws_dir + '/ur10_arm/robot_description_test.yaml',
        urdf_path=ws_dir + '/ur10_arm/ur10.urdf',
        rmpflow_config_path=rmpflow_config,
        end_effector_frame_name='gripper_base',
        maximum_substep_size=0.00334
    )

    articulation_rmpflow = ArticulationMotionPolicy(arm_art, rmpflow)


def compute(db: og.Database):
    global arm_art, rmpflow, articulation_rmpflow, target

    if arm_art is None or rmpflow is None or articulation_rmpflow is None or target is None:
        db.log_error("Motion system is not initialized. Please trigger setup first.")
        return False

    # Get target pose
    target_position, target_orientation = target.get_world_poses()
    target_position = target_position[0]
    target_orientation = target_orientation[0]
    # print(f"Target Pos: {target_position},  Target Orientation(quat): {target_orientation}")

    # Set target pose in RMPFlow
    rmpflow.set_end_effector_target(target_position, target_orientation)
    rmpflow.update_world()

    # Set robot base pose
    robot_base_translation, robot_base_orientation = arm_art.get_world_pose()
    rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)

    # Compute and apply next action
    action = articulation_rmpflow.get_next_articulation_action(step)
    arm_art.apply_action(action)

    return True


def cleanup(db: og.Database):
    pass







# class UR10ArmRMPFlow():
#     def __init__(self):
#         # rmpflow params
#         self._rmpflow = None
#         self._articulation_rmpflow = None
#         # system/robot params
#         self._articulation = None
#         # visualization params
#         self._plan_visual = None
#         # target pose params
#         self._target = None

#     def load_assets(self):
#         pass

#     def setup_assets(self):
#         robot_prim_path = "/World/ur10_short_suction"
#         robot_art = Articulation(robot_prim_path)
#         self._articulation = robot_art

#         target_prim_path = "/World/target"
#         target_prim = XFormPrim(target_prim_path)
#         self._target = target_prim

#         # RMP-Flow default directory in isaacsim
#         mg_extension_path = home_dir+"/labeeb/isaac_sim_45/exts/isaacsim.robot_motion.motion_generation"
#         rmpflow_config_dir = os.path.join(mg_extension_path, 'motion_policy_configs')
#         rmpflow_config = rmpflow_config_dir + "/ur10/rmpflow/ur10_rmpflow_config.yaml"

#         # setting up rmpflow
#         self._rmpflow = RmpFlow(
#             robot_description_path=ws_dir+'/ur10_arm/robot_description_test.yaml',
#             urdf_path=ws_dir+'/ur10_arm/ur10.urdf',
#             rmpflow_config_path=rmpflow_config,
#             end_effector_frame_name='gripper_base',
#             maximum_substep_size = 0.00334
#         )

#         # connecting rmpflow motion policy to the ur10 arm articulation motion policy
#         self._articulation_rmpflow = ArticulationMotionPolicy(arm_art, rmpflow)
    
#     def update(self, step: float):
#         target_position, target_orientation = self._target.get_world_poses() # wrt sim world frames
#         self._rmpflow.set_end_effector_target(
#             target_position, target_orientation
#         )

#         self._rmpflow.update_world()
#         robot_base_translation, robot_base_orientation = self._articulation.get_world_poses()
#         self._rmpflow.set_robot_base_pose(
#             robot_base_translation, robot_base_orientation
#         )

#         action = self._articulation_rmpflow.get_next_artiulation(step)
#         print(f"Taking action: {action}")
#         self._articulation.apply_action(action)

#     def reset(self):
#         pass







