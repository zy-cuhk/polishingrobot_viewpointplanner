// the function of this cpp is to generate a viewpoint for camera and then obtain the covered octomap and visualize the octoma
// step one: generate a viewpoint 
// step two: obtain the covered octomap 
// step three: visualize the changed octoamp in RVIZ


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>

#include <ros/ros.h>  
#include "ros/console.h"
#include <sensor_msgs/PointCloud2.h>  

#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;

int main(int argc, char** argv)
{
    // ros node initialization
	ros::init (argc, argv, "viewpoint_planning_using_octomap");  
	ros::NodeHandle nh;  

    // set up pcl rostopic and transform pcd to point cloud 
	std::string topic,path,frame_id;
    int hz=5;
    nh.param<std::string>("path", path, "/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_viewpointplanner/src/test_pcd.pcd");
	nh.param<std::string>("frame_id", frame_id, "map");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic, 10);  
	sensor_msgs::PointCloud2 output;  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud); 
	pcl::toROSMsg(*cloud,output);
	output.header.stamp=ros::Time::now();
	output.header.frame_id  =frame_id;
	cout<<"path = "<<path<<endl;
	cout<<"frame_id = "<<frame_id<<endl;
	cout<<"topic = "<<topic<<endl;
	cout<<"hz = "<<hz<<endl;
	ros::Rate loop_rate(hz);

    // set up octomap rostopic 
    bool isLatch = false;
    auto octomapPublisher = nh.advertise<octomap_msgs::Octomap>("octomap", 1, isLatch);
    octomap_msgs::Octomap octomapMsg;


    // create octree from point cloud 
    float cloudCentroid[3]={0,0,0};
    octomap::OcTree cloudAndUnknown(0.05);
    for (size_t i = 0; i < cloud->points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2],true);
        cloudNode->setValue(13);
    }

    //visualize pcd document octree
    octomap::Pointcloud scan;
    octomap::OcTree tree(0.05);
    octomap::point3d sensorOrigin(0,0,0);
    for (size_t i =0; i < cloud->points.size (); ++i){
        scan.push_back(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2]);
    }
    tree.insertPointCloud(scan, sensorOrigin);


    // create FOV Of camera
    octomap::point3d Point3dwall(1.05,0.0,0.0);    
    octomap::Pointcloud pointwall;     
    // for(int ii=1;ii<321;ii++){
    for(int ii=1;ii<401;ii++){
        for(int iii=1;iii<241;iii++){
            Point3dwall.y()= (-0.690027)+(ii*0.003489);
            // Point3dwall.y()= (-0.560027)+(ii*0.003489);
            Point3dwall.z()= (-0.430668)+(iii*0.003574);
            pointwall.push_back(Point3dwall);
        }
    }


    bool flag1;
    for (int time=0;time<3;time++){
        // change camera pose
        octomap::Pointcloud variablePointwall;     
        octomap::point3d iterator; 
        iterator.x()=0.0;        
        iterator.y()=0.0;  
        iterator.z()=0.430668+time*0.85776;
        
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
        octomath::Quaternion Rotation2(0,0.0,0.0);	
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
        variablePointwall=pointwall;		
        variablePointwall.transform(RotandTrans2);

        // add and visualize fov octree
        // tree.insertPointCloud(variablePointwall,sensorOrigin);    
        // tree.writeBinary("check.bt");  

        // raycast to obtain voxel
        octomap::KeyRay rayBeam;
        int unknownVoxelsInRay=0;
        int known_points_projection=0;
        for (int ii=0; ii<variablePointwall.size();ii++){
            bool Continue=true;		
            cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode * node=cloudAndUnknown.search(*it);	
                if(node!=NULL){
                    // if (node->getValue()==13){
                    cloudAndUnknown.updateNode(*it, false);
                    
                    Continue=false;
                    //}
                    // if (node->getValue()==13){
                    //     // std::cout<<"the position is:"<<rayBeam.begin()<<std::endl;
                    //     flag1=cloudAndUnknown.updateNode(*it, false);
                    //     // node->setColor(255,255,0);
                    //     node->setValue(24);
                    //     // flag1=node->getOccupancy();
                    //     std::cout<<"the updation state is: "<<flag1<<std::endl;
                    //     known_points_projection++;
                    //     Continue=false;
                    // }

                }
            }
        }
		cloudAndUnknown.updateInnerOccupancy();
        // cloudAndUnknown.writeBinary("check.bt");  
    

        // // compute the node which has been changed values
        // octomap::point3d iterator1; 
        // int coverage_points_num=0;
        // int uncoverage_points_num=0;
        // for (size_t i = 0; i < cloud->points.size (); ++i){ 
        //     iterator1.x()=cloud->points[i].x+cloudCentroid[0];
        //     iterator1.y()=cloud->points[i].y+cloudCentroid[1];
        //     iterator1.z()=cloud->points[i].z+cloudCentroid[2];
        //     octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
        //     if(node1!=NULL){
        //         if (node1->getValue()==24){
        //             coverage_points_num++;
        //         }
        //         if (node1->getValue()==13){
        //             uncoverage_points_num++;
        //         }
        //     }
        // }
        // std::cout<<"the points number of wall is: "<<cloud->points.size ()<<std::endl;
        // std::cout<<"the coverage points number is: "<<coverage_points_num<<std::endl;
        // std::cout<<"the uncoverage points number is: "<<uncoverage_points_num<<std::endl;
        // std::cout<<"the points number of camera is: "<<variablePointwall.size()<<std::endl;
        // std::cout<<"projected known voxel number is: "<<known_points_projection<<std::endl;
        // std::cout<<"---------------------------------"<<std::endl;

        // ros loop for octomap occupancy
        octomapMsg.header.frame_id = "map";
        octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
        //while (ros::ok())
        //{
            octomapPublisher.publish(octomapMsg);
            pcl_pub.publish(output);  
            loop_rate.sleep();  
            //ros::spinOnce();  
        //}

        }



    // // ros::spin();
    return 0;

}
