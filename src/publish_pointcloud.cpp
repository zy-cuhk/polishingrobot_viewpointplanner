#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>

using namespace std;


int main (int argc, char **argv)  
{  
	ros::init (argc, argv, "publish_pointcloud");  
	ros::NodeHandle nh;  

	std::string topic,path,frame_id;
    int hz=5;
    nh.param<std::string>("path", path, "/home/zy/catkin_ws/src/polishingrobot_ylz/publish_pointcloud/data/test_pcd.pcd");
	nh.param<std::string>("frame_id", frame_id, "map");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);
   
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic, 10);  

	sensor_msgs::PointCloud2 output;  
	// the original version
	// pcl::PointCloud<pcl::PointXYZ> cloud;  
	// pcl::io::loadPCDFile (path, cloud);  
	// pcl::toROSMsg(cloud,output);
	// the second version
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

	while (ros::ok())  
	{  
		pcl_pub.publish(output);  
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	return 0;  
}  
