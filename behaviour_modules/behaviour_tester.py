import carb # type: ignore
from omni.kit.scripting import BehaviorScript # type: ignore
import omni.kit.window.property # type: ignore
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS #type: ignore
from isaacsim.replicator.behavior.utils.behavior_utils import ( # type: ignore
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)

from isaacsim.core.prims import RigidPrim # type: ignore
from pxr import Usd, Sdf, Gf, UsdGeom # type: ignore

import numpy as np

class MoveBoxBehaviour(BehaviorScript):
    BEHAVIOUR_NS = "move_box_behaviour"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 0,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
    ]

    def on_init(self):
        self._update_counter = 0
        self._interval = 0
        
        self._valid_prims = []
        self.cb_rigid_prim = None
        self._rigid_prim_initialized = False

        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info("on destroy")
        self._reset()
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOUR_NS, self.VARIABLES_TO_EXPOSE)
            omni.kit.window.property.get_window().request_rebuild()

    def on_play(self):
        carb.log_info("on play")
        self._setup()
        if self._interval > 0:
            carb.log_info(f"hello")

    def on_pause(self):
        carb.log_info("script paused")

    def on_stop(self):
        carb.log_info("behaviour script stopped !")
        self._reset()
        carb.log_info(f"on_stop()->{self.prim}")

    def on_update(self, current_time: float, delta_time: float):
        if delta_time <= 0:
            return

        if self._interval <= 0:
            pass
        else:
            self._update_counter +=1
            
            # running the behaviour/logic every self._interval
            if self._update_counter >= self._interval:
                print(f"on_update({current_time}, {delta_time}, {1/delta_time})")




                self._update_counter = 0

    def _setup(self):
        carb.log_info(f"setting up prims and script variables")
        self._interval = self._get_exposed_variable("interval")
        # get prims
        self._valid_prims = [prim for prim in Usd.PrimRange(self.prim) if self.prim.IsValid()]
        for i, prim in enumerate(self._valid_prims):
            print(f"child prim {i+1}: {prim}")

        # wrapping prim in rigid prim api to expose rigid prim properties for ease
        if not self._rigid_prim_initialized:
            prim_path = self.prim.GetPath().pathString
            self.cb_rigid_prim = RigidPrim(
                prim_paths_expr=prim_path,
                name="cb1_rigid_prim_view",
            )
            self._rigid_prim_initialized = True
            carb.log_info(f"Cardbox_B1 rigid prim initialized at prim path: {prim_path}")

        if not self.prim.IsValid():
            carb.log_info("prim not valid yet!")

    def _reset(self):
        self._valid_prims.clear()
        self._rigid_prim_initialized = False
        self.cb_rigid_prim = None
        self._update_counter = 0
        self._interval = 0

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOUR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)
    
    # helper functions/attributes/methods
    def _get_location(self):
        pass


