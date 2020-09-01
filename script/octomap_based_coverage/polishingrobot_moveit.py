#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import tf


from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

def mobile_platform_rviz_motion(mobileplatform_targetjoints):
    "def of mobile platform motion groups" 
    mobileplatform = moveit_commander.MoveGroupCommander('mobile_platform')
    
    "motion of mobile platform" 
    print("mobileplatform_targetjoints=",mobileplatform_targetjoints)
    mobileplatform.set_joint_value_target(mobileplatform_targetjoints)
    mobileplatform_state=mobileplatform.go()
    rospy.sleep(0.2)
    if mobileplatform_state==False:
        rospy.logerr("mobile platform planning is failed !")
    return mobileplatform_state

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('polishingrobot_moveit', anonymous=True)

    mobileplatform_targetjoints=[1,1,0,0,0,0]
    "def of mobile platform motion groups" 
    mobileplatform = moveit_commander.MoveGroupCommander('aubo10')
    try:
        # while not rospy.is_shutdown():               
        "motion of mobile platform" 
        print("mobileplatform_targetjoints=",mobileplatform_targetjoints)
        mobileplatform.set_joint_value_target(mobileplatform_targetjoints)
        mobileplatform_state=mobileplatform.go()
        rospy.sleep(0.2)
        # mobileplatform_state=mobile_platform_rviz_motion(mobileplatform_targetjoints)            
    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)




