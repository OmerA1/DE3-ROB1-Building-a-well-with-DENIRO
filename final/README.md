# Please follow the following instructions to make your Gazebo simulations more accurate

## Change DENIRO's base friction:
~$ cd grasping_ws/src/baxter_common/baxter_description/urdf/pedestal

replace the "quickie.xacro" file with the one in this folder

## Change DENIRO's Gripper size:
~$ cd grasping_ws/src/baxter_common/baxter_description/urdf

open "left_end_effector.urdf.xacro"

change BOTH "l_finger_slot='2'" to "l_finger_slot='4'"

*do the same for "right_end_effector.urdf.xacro" if using both grippers*

## To run the .py code:
~$ roslaunch baxter_gazebo baxter_world.launch

~$ cd "*to the directory of the .py code*"

~$ python "*the code.py*"
