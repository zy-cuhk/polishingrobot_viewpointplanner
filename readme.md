For the package of polishingrobot_viewpointplanner

------------------------------------------------------------------------------------------
The first version of algorithm, the actions to be taken include:
1. roslaunch polishingrobot_viewpointplanner viewpoint_planning.launch
2. matlab run: cartesianspace_viewpoints_generation.m
3. python run: rosrun polishingrobot_viewpointplanner jointspace_viewpoints_computation.py
4. matlab run: effective_cartesianspace_viewpoints_visualization.m


-------------------------------------------------------------------------------------------
The reviews of second version of algorithm are shown as follows:

1. the keywords: "view planning", "coverage planning", "coverage path planning" for searching the code in github for camera veiwpoint planning problem, however no code related with octomap code has been found for this problem.

2. point cloud based coverage viewpoint planning: https://github.com/search?p=1&q=next+best+view&type=Repositories, the reference paper is: 
Global Registration of Mid-Range 3D Observations and Short Range Next Best Views,

3. octomap based coverage viewpoint planning: https://github.com/JoseJaramillo/NBV/blob/master/main.cpp, the reference paper is: 
This algorithm reads a PLC file and prints the Next Best View (NBV) coordinates based on the Occlusion Aware VI method presented in [1]. The resulting coordinates are computed in 2D, further updates will include 3D NBVs. The camera is set up as a kinect 1.
[1] Delmerico, J., Isler, S., Sabzevari, R. et al. Auton Robot (2018) 42: 197. https://doi.org/10.1007/s10514-017-9634-0

4. point cloud based coverage viewpoint planning: https://github.com/RMonica/surfel_next_best_view/tree/master/surfel_next_best_view
the reference paper is: R. Monica and J. Aleotti, "Surfel-Based Next Best View Planning", in IEEE Robotics and Automation Letters, vol. 3, no. 4, pp. 3324-3331, Oct. 2018

5. octomap based coverage viewpoint planning: https://github.com/ethz-asl/nbvplanner,
the refrence paper is: @inproceedings{bircher2016receding, title={Receding horizon "next-best-view" planner for 3D exploration}, author={Bircher, Andreas and Kamel, Mina and Alexis, Kostas and Oleynikova, Helen and Siegwart, Roland}, booktitle={2016 IEEE International Conference on Robotics and Automation (ICRA)}, pages={1462--1468}, year={2016}, organization={IEEE} }

-----------------------------------------------------------------------------------------------
Generally the programming approaches can be divided into three types based on the 3d format of scanning object:
1. cloud point based method 
2. octomap based method
3. mesh based method 











 


























