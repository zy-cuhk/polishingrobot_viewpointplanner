#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import numpy
import os,sys
import socket

from aubo_kienamatics import *
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import numpy.matlib

from scipy.io import loadmat
class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.sub_state_ = rospy.Subscriber("/joint_states", JointState, self.sub_state, queue_size=10)
        self.aubo_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]

    def Init_node(self):
        rospy.init_node("aubo_jointvalue_viewpoints_generation")
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
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'yue'
        js.name = ["base_joint1", "base_joint2","mobilebase_joint","rodclimbing_joint","shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)
    def sub_state(self,msg):
        self.aubo_joints_value[0]=msg.position[4]
        self.aubo_joints_value[1]=msg.position[5]
        self.aubo_joints_value[2]=msg.position[6]
        self.aubo_joints_value[3]=msg.position[7]
        self.aubo_joints_value[4]=msg.position[8]
        self.aubo_joints_value[5]=msg.position[9]
    def aubo_reachjoints(self, pub_joints_value):
        if self.aubo_joints_value==pub_joints_value:
            flag=True
        else:
            flag=False
        return flag

def main():
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    ratet=2
    rate = rospy.Rate(ratet)

    temp=[0.0,0.0,0.0,0.0]
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(temp+q_ref)
    
    
    Aub.pub_state(q_ref_rad)


    Aubo10=Aubo_kinematics()
    mat = loadmat("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_viewpointplanner/script/data.mat")
    viewpoints_num_count=1
    q_sol_without_collistion_dict={}
    viewpoints_dict={}
    # for i in range(1):
    #     for j in range(1):
    for i in range(len(mat["camera_viewpoints"])):
        for j in range(len(mat["camera_viewpoints"][0])):
            print("----------------------------------------------------")
            print("the viewpoints number is:",viewpoints_num_count)
            trans=mat["camera_viewpoints"][i,j,0:3]
            T_matrix=Aub.addtrans(Aubo10.aubo_forward(q_ref),trans)
            q_dict=Aubo10.GetInverseResult_without_ref(T_matrix)
            # print("q_dict",q_dict)
            not_collision_qlist=[]
            if q_dict!=None:
                collision_count=0
                for m in range(len(q_dict)):
                    Aub.pub_state(temp+q_dict[m])
                    pub_joints_value=q_dict[m]
                    rate.sleep()
                    while(1):
                        Aubo_state=Aub.aubo_reachjoints(q_dict[m])
                        if Aubo_state==True:
                            judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')
                            if judge_self_collision_flag==False:
                                not_collision_qlist=not_collision_qlist+q_dict[m]
                                rospy.logerr("the joints value is not in collison")
                                print(q_dict[m])
                            else:
                                collision_count=collision_count+1
                            if collision_count==len(q_dict):
                                rospy.logerr("all viewpoints at this viewpoints are collision")
                            # q_sol_without_collistion_dict.update({viewpoints_num_count:not_collision_qlist})
                            break
            # else:
            q_sol_without_collistion_dict.update({viewpoints_num_count:not_collision_qlist})
            viewpoints_dict.update({viewpoints_num_count:trans})
            viewpoints_num_count=viewpoints_num_count+1
    print(q_sol_without_collistion_dict)
    print("------------------------------------")
    print(viewpoints_dict)

    ratet=0.5
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        for i in range(len(q_sol_without_collistion_dict)):
            list1=q_sol_without_collistion_dict[i+1]
            if len(list1)!=0:
                print("list1 is:",list1)
                q_list=list1[0:6]
                q_list=temp+q_list
                Aub.pub_state(q_list)
                rate.sleep()
    

    

if __name__ == '__main__':
    main()