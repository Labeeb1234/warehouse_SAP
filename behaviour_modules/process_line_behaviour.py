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
from isaacsim.core.api.materials import OmniPBR, VisualMaterial # type: ignore
from pxr import Usd, Sdf, Gf, UsdGeom # type: ignore

import numpy as np

class MoveBoxBehaviour(BehaviorScript):
    BEHAVIOUR_NS = "move_box_behaviour"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 10,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
        {
            "attr_name": "start_process",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": False,
            "doc": "To Mimic input processing online"
        },
    ]

    default_material = OmniPBR(
        prim_path="/World/warehouse_imports/Cardbox_B1/Looks/Cardboard_B1"
    )
    green_material_color = OmniPBR(
        prim_path="/World/warehouse_imports/Cardbox_B1/Looks/PCB_B1",
        color=np.array([0.0, 0.4, 0.0]),
    )
    green_material_color.set_reflection_roughness(0.1)  # glossy surface
    green_material_color.set_metallic_constant(0.0) 

    def on_init(self):
        carb.log_info("Initializing Behaviour Script...")
        self._update_counter = 0
        self._interval = 0
        self._start_process = False

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
                print(f"on_update(t:{current_time}, dt:{delta_time}, physics_updr:{1/delta_time})")

                if self._start_process:
                    print(f"entering...")
                    if current_time > 8.0 and current_time <= 8.1:
                        self.cb_rigid_prim.apply_visual_materials(self.green_material_color)

                self._update_counter = 0

    def _setup(self):
        carb.log_info(f"setting up prims and script variables")
        self._interval = self._get_exposed_variable("interval")
        carb.log_info(f"current behaviour script update interval: {self._interval}")
        self._start_process = self._get_exposed_variable("start_process")

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
            # setting default color on setup
            self.cb_rigid_prim.apply_visual_materials(self.default_material)


        if not self.prim.IsValid():
            carb.log_info("prim not valid yet!")

    def _reset(self):
        if self.cb_rigid_prim and self.cb_rigid_prim.prims:
            self.cb_rigid_prim.apply_visual_materials(self.default_material)
            carb.log_info("Reset to default material.")

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

    def _cleanup_old_variables(self):
        OLD_BEHAVIOUR_NS = "custom_script"
        OLD_VARIABLES = [
            {
                "attr_name": "interval",
                "attr_type": Sdf.ValueTypeNames.UInt,
                "default_value": 0,
                "doc": "Old interval variable",
            },
        ]
        remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, OLD_BEHAVIOUR_NS, OLD_VARIABLES)
        omni.kit.window.property.get_window().request_rebuild()
    
    def check_if_old_variables_exist(self, prim, old_ns, variable_defs):
        prefix = f"{old_ns}"
        attr_names = [f"{prefix}{v['attr_name']}" for v in variable_defs]

        found_attr = []
        for attr_name in attr_names:
            if prim.HasAttribute(attr_name):
                found_attr.append(attr_name)
        
        return found_attr


