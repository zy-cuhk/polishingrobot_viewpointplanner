#include <fstream>

#include "ros/console.h"
#include "ros/ros.h"

#include <octomap/OcTreeBase.h>
#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


enum class SendType { BINARY, FULL, AUTO };
enum class TreeType { BINARY_TREE, OC_TREE, INVALID_TREE };

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "hello_world_map_server", ros::init_options::AnonymousName);

    std::string filename;
    filename="/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_viewpointplanner/src/test_pcd.bt";
    bool isLatch = false;

    
    ros::NodeHandle nodeHandle; 
    auto octomapPublisher = nodeHandle.advertise<octomap_msgs::Octomap>("octomap", 1, isLatch);
    octomap_msgs::Octomap octomapMsg;

    std::cout<<"yes, this is the debug position"<<std::endl;
    auto ocTree = octomap::OcTree(filename);
    octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);

    while (1)
    {
        octomapMsg.header.frame_id = "map";
        octomapPublisher.publish(octomapMsg);
        ROS_INFO("OK. Message length = %d bytes", octomapMsg.data.size());
    }
    ros::spin();

    return 0;
}
