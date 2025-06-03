# Warehouse SAP Simulation Documentation

### 🛠/📦 System Requirements and Dependencies
--------------------------------------------------------------

- OS: Ubuntu 22.04 (can work on 24.04 not 24.1)
- Software: NVDIA Isaacsim (4.5.0)
- Progamming Language: Python
- Other dependencies required are installed automatically when IsaacSim base is installed in the system 

### Tasks (SMTLine assmebly line functionality)
--------------------------------------------------------------

- Mesh selection (done)
- Fix Mesh (done)
- Mesh wrapping online (done)
- Behaviour Script for process line (done --> can be improved as of now on hold)
- model selection for arm (done selected UR10 arm with a suction gripper)
- Test out cortex framwork for arm routine (done)
 ----> (USING STATE MACHINE TO MODEL THE ROUTINE TASKS)
- Created a null space tester code in state machine model (done) --> for testing purposes
- integrated rmpflow (done)
- need to check cortex framework app with a custom USD env, the warehouse env USD ---> (done successful)

### Tasks (Developing the pick and place routine using DECIDER STATE MACHINE)
----------------------------------------------------------------

- creating reach to pose state using base rmpflow motion planning (done subject to changes based on the env and robot description file and URDF)
- creating homing state (done)
- creating reach to place state (ongoing)
- creating some helper states (status based on requirement, as of now done)
- creating status checker and logical state monitor class to monitor feedbacks (progress dependent on other tasks)
- creating on and off states for gripper (done) ---> untested
- creating state_machine_deicder (ongoing --> changed based env loaded)
- creating Decider Network using state machines (future improvement)


### Current Setup
-------------------------------------------------------------

<div align="centre">
 <img src="https://github.com/user-attachments/assets/34cfebce-2f04-4c88-a094-68cecf4b6dd7" alt="STMLine warehouse setup"/>
</div>

- The USD file to be used [here](https://github.com/Labeeb1234/warehouse_SAP/blob/main/warehouse_setup/new_warehouse_setup_small.usd)
- The arm models are directly referenced from the IsaacSim beta assets (assesible from the GUI as of 2025), the other smalled imports are from the NVIDIA IsaacSim database also assesible from the GUI as of 2025.
- **PS**: - There are chances of material dependency issues for the conveyor models as I had to locally install the materials for that particular model, for some reason IsaacSim was not able to detect it automatically from the omniverse link.

### Documentation
-------------------------------------------------------------

- Starting out all the tasks and the main framework and software used for this warehouse simualtion, which maybe extended to a digital twin simulation for purposes currently beyond the scope of this documentation
- Initially the warehouse layout was created using out own blueprint and scale  (image given below) based on a PCB assembly based factory, eventually figured out the required assets for the required scale is hardly present anywhere by default so we scaled down the enviroment with the setup pic as shown in the previous section.
<div align="centre">
 <img src="https://github.com/user-attachments/assets/da691860-44eb-4021-bb8f-f10d6aa90420" alt="Initial Setup"/>
</div>


## ===========================================================
### Note:  Check the old model descriptions for more info on those file.
-------------------------------------------------------------
- **Note to Self**: No commands to run as of now just an automatically running behaviour script for the assembly process mimicking; need to integrate the arm **decider_network-FSM** for pick and place routine.
