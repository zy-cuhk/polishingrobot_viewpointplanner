For the package of polishingrobot_viewpointplanner


## the first version of algorithm, the actions to be taken include:
1. roslaunch polishingrobot_viewpointplanner viewpoint_planning_no_octomap.launch
2. matlab run: cartesianspace_viewpoints_generation.m
3. python run: rosrun polishingrobot_viewpointplanner jointspace_viewpoints_computation.py
4. matlab run: effective_cartesianspace_viewpoints_visualization.m
# the function is to verify collision check of moveit and simple grid based coverage method 


## the second version of algorithm, the actions to be taken include:
1. roslaunch polishingrobot_viewpointplanner viewpoint_planning_with_octomapserver.launch 
# the function is to obtain collision check of moveit and octomap visualization with typical octomap server


## the third verision of algorithm, the actions to be taken include:
1. rosrun polishingrobot_viewpointplanner pubilish_pointcloud
2. rosrun polishingrobot_viewpointplanner collision_check

# the function is to obtain collision check of moveit and modified octomap 











 


























