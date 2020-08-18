#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
# from aubo_robotcontrol import *
import time
import numpy
import os
import socket
from aubo_kienamatics import *
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import numpy.matlib
import json

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


from scipy.io import loadmat
class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.M=0
        self.N=0
    def Init_node(self):
        rospy.init_node("aubo_collision_cehck")
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * pi / 180)
        return tuple(dd)
    def rad_to_degree(self,tuplelist):
        dd=[]
        for i in tuplelist:
            dd.append(i*180/math.pi)
        return dd
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'yue'
        js.name = ["base_joint1", "base_joint2","mobilebase_joint","rodclimbing_joint","shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)
    def addtrans(self,inital_list_data,list_data):
        temp=[]
        for i in range(len(inital_list_data)):
            if i==3:
                temp.append(list_data[0])
            elif i==7:
                temp.append(list_data[1])
            elif i==11:
                temp.append(list_data[2])
            else:
                temp.append(inital_list_data[i])
        return temp


def main():
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    ratet=1
    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)

    Aubo10=Aubo_kinematics()
    m = loadmat("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_onlineplanner/src/data1.mat")

    count=0
    count_o=0
    num_count=0
    judge_count_flag=0

    q_sol_without_collistion_dict={}
    judge_self_collision_list=[]

    judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')
    rospy.logerr("judge =="+str(judge_self_collision_flag))

    judge_self_collision_list.append(judge_self_collision_flag)
    last_state=[]

    last_data_degree=[]
    last_data_rad=[]
    use_ref_rad_onece_flag=0

    flag=1
    while not rospy.is_shutdown():
        if flag==1:
            temp1=[0.0,0.0,0.0,0.0]
            Aub.pub_state(temp1+q_ref_rad)
            flag=0
        else:
            if count_o<len(m["camera_viewpoints_inlowerbaseframe"]):
                T_matrix=Aub.addtrans(Aubo10.aubo_forward(q_ref),m["camera_viewpoints_inlowerbaseframe"][count_o])
                q_dict=Aubo10.GetInverseResult_without_ref(T_matrix)
                
                if q_dict!=None:
                    if count< len(q_dict):
                        temp=[0.0,0.0,0.0,0.0]
                        Aub.pub_state(temp+q_dict[count])
                        judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')

                        if count!=0:
                            rospy.loginfo("haha--> "+str(count_o))
                            rospy.logerr("last state pub judge =="+str(judge_self_collision_flag)+" "+str(q_dict[count-1]))
                        
                            if judge_self_collision_flag==False:
                                rospy.logerr("The o last point : "+str(count_o)+" judge_self_collision sol "+str(count-1)+" ["+str(judge_self_collision_flag)+"]")
                                q_sol_without_collistion_dict.update({num_count:q_dict[count-1]})
                                num_count+=1
                        count+=1
                        
                    else:
                        count=0
                        count_o+=1
                        num_count=0
                        if len(q_sol_without_collistion_dict)!=0:
                            if use_ref_rad_onece_flag==0:
                                rets,q_choose=Aubo10.chooseIKonRefJoint(q_sol_without_collistion_dict,q_ref_rad)
                                use_ref_rad_onece_flag=1
                            else:
                                rets,q_choose=Aubo10.chooseIKonRefJoint(q_sol_without_collistion_dict,last_data_rad[-1])
                            rospy.logerr("q_choose"+str(q_choose))
                            print(Aub.rad_to_degree(q_choose))
                            last_data_rad.append(q_choose)
                            last_data_degree.append(Aub.rad_to_degree(q_choose))
                        q_sol_without_collistion_dict={}
                        # break
                else:
                    count_o+=1
            else:
                print(last_data_rad) 
                print(last_data_degree)   
                
        rate.sleep()
            

if __name__ == '__main__':
    main()