# !/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import numpy.matlib

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from scipy.io import loadmat

wall2mobilebase_distance=1.0
def rectangle_plot(horizontal_value,vertical_value,i,j):
    A=[]

def main():
    ## obtain point A B C D of effective workspace in mobile platfrom cooridinate frame 
    data = loadmat("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_viewpointplanner/script/data.mat")
    polishing_effective_workspace=data["region_points"]
    print(polishing_effective_workspace[3,2])

    ## determine the FOV of camera 
    wall2camera_distance=0.80
    camera_fov_length=wall2camera_distance/0.80*0.54
    camera_fov_width=camera_fov_length*2/3

    ## the workspace is divided as follows:
    region_height=max(polishing_effective_workspace[:,2])-min(polishing_effective_workspace[:,2])
    region_width=max(polishing_effective_workspace[:,1])-min(polishing_effective_workspace[:,1])
    M=int(math.ceil(region_height/camera_fov_length))
    N=int(math.ceil(region_width/camera_fov_width))
    print(M,N)

    horizontal_value=np.zeros(N+1)
    for i in range(N+1):
        horizontal_value[i]=min(polishing_effective_workspace[:,1])+i*camera_fov_width
        if i==N:
            horizontal_value[i]=max(polishing_effective_workspace[:,1])

    vertical_value=np.zeros(M+1)
    for i in range(M+1):
        vertical_value[i]=max(polishing_effective_workspace[:,2])-i*camera_fov_length
        if i==M:
            vertical_value[i]=min(polishing_effective_workspace[:,2])
    print("horizontal_value is:", horizontal_value)
    print("vertical_value is:", vertical_value)
    

    



    


    
if __name__ == "__main__":
    main()