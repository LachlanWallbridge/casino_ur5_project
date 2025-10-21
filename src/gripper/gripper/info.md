This code is taken from Davids repo. As the end effector is so similar, I have placed it in here as a guide.

`gripper_server.py` uses the `GripperCmd.srv` and `ResetGripperCmd.srv` services to drive the end effector. We will need to modify this to work with the arduino.

`gripper_client.py` give us a CLI to test the gripper.

Both these and the services need to be change to work with our robot. `end_effector_description` is also taken from the same repo. We will need to make this our own. I have included this as it is HIGHLY important for tuning MoveIt. Please be careful when changing this, don't remove the ur5 intergration as it is required for tuning moveit.

NOTE: The `end_effector_description` currently has a launch file in the Cmake. We may want to change this to be in the brain. Steps to do:
- change `launch rviz urdf etc meshes` to just `urdf`

Additionally, make sure we change everything to ur5 instead of ur10