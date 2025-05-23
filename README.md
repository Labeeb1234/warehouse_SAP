- Framwork: Isaacsim(4.5.0) - Python

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


## ===========================================================
### Note:  Check the old model descriptions for more info on those file
