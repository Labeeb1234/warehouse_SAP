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
- **PS**: - There are chances of material dependency issues for the conveyor models as I had to locally install the materials for that particular model, for some reason IsaacSim was not able to detect/download it automatically from the omniverse database.

### Documentation
-------------------------------------------------------------

warehouse layout was initially designed using a **custom blueprint and scale**, inspired by a **PCB assembly factory workflow**.
- During development, it became apparent that **ready-made assets at the required industrial scale were scarce**.
- To ensure rapid prototyping and functional validation, the environment was **scaled down proportionally**, while preserving spatial relationships and task semantics.

### Initial Setup Reference

<div align="center">
  <img src="https://github.com/user-attachments/assets/da691860-44eb-4021-bb8f-f10d6aa90420" 
       alt="Initial Setup" height="512" width="512"/>
</div>

---

## Experimental Results and Observations

> ⚠️ *Note: This section documents early experimental outcomes and will be reorganized further as the project matures.*

### Isolated Manipulation and Object Setup

- The robotic arm and object interactions were **isolated from the full warehouse environment** to validate the behavioral framework independently.
- This approach allowed focused testing of motion planning, task sequencing, and feedback-driven execution.

---

## Behavioral Framework: Cortex (Isaac Sim)

- The **Cortex framework** in Isaac Sim was used to structure robotic tasks.
- Cortex enables decomposition of **complex, multi-step, repetitive tasks** into:
  - **State machines**
  - **Context-aware state monitors**
- This design significantly improves task robustness by allowing real-time feedback and conditional transitions.

---

## Motion Generation

- The base motion planner used is **RMPFlow**, Isaac Sim’s default **policy-based motion generation framework**.

### Key Capabilities

- End-effector (EEF) pose control using:
  - Target **position**
  - **Orientation**
  - **Approach bias**
  - **Joint posture bias**
- Real-time **obstacle avoidance**, conceptually similar to MoveIt2’s planning scene.
- Obstacles can be added by:
  1. Adding object prims to the stage
  2. Registering them with the motion generation context via API calls

📎 Reference:  
[IsaacSim_Motion_Generation_Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/concepts/index.html)

---

## State Machine Experiments

### Singularity Testing

- As an initial validation of state-machine-driven control, a **singularity test** was performed.
- The test involved commanding the robot to reach the **same end-effector pose** using **different joint configurations**, forcing the arm to pass through **null-space planes**.
- This validated:
  - Configuration-space diversity
  - Stability of motion generation near singular regions

<div align="center">
  <img src="https://github.com/user-attachments/assets/4b00670c-6c7a-4500-815a-7dfc985fecbf" 
       alt="Singularity Testing"/>
</div>

---

### Target Pose Reach Test

- A state-machine-controlled task was implemented to command the arm to reach a specified **target pose**.
- A **context monitor** continuously evaluated whether the end-effector had successfully reached the goal.
- State transitions were triggered based on sim-time feedback from the monitor.

<div align="center">
  <img src="https://github.com/user-attachments/assets/94f77860-6ff0-47a4-a6b8-212756888129" 
       alt="Go-to Target Pose Testing"/>
</div>

---

### Pick And Place Task (In Isolated Environment) 

- Sequence of state-machine-controlled **pick and place task** was implemented and tested on the same setup.
- The entire structure only uses context-monitors state machines to transition from one state to another (may not be that reponsive).

<div align="centre">
  <img src="https://github.com/user-attachments/assets/c3f09a5a-5697-409c-b14c-0b47796cac7a" 
       alt="State-Machine only pick and place task"/>
</div>

---

  




## ===========================================================
### Note:  Check the old model descriptions for more info on those file.
-------------------------------------------------------------
- **Note to Self**: No commands to run as of now just an automatically running behaviour script for the assembly process mimicking; need to integrate the arm **decider_network-FSM** for pick and place routine.
