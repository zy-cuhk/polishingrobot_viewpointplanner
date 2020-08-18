polishing robot online planner is functioned as a viewpoint planner

step 1: Cartesian-space viewpoints list generation 
step 2: Joint-space viewpoints list generation 


the actions to be taken include:
1. roslaunch polishingrobot_onlineplanner planning_scene.launch 
2. roslaunch polishingrobot_moveit_config demo.launch
3. rosrun polishingrobot_onelineplanner aubo_collision_bk.py

----------------------------------------------------------------------------------------------------------------
1. the first program is: polishing_effective_workspace_computation.py, the effective workspace is defined as a rectangle consisting of A B C D points. 
the input is the distance between wall plane and mobile platform coordinate frame 
the output is A B C D points Cartesian-space position in the mobile platform coordinate frame 
----------------------------------------------------------------------------------------------------------------
2. the second program is: cartesian_space_viewpoints_computation.py
the input is  A B C D points Cartesian-space position in the mobile platform coordinate frame 

2.1 the rectangle is divided into several rectangles, which can be 



