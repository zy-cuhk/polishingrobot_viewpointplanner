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

    // const auto ocTreeBase = std::shared_ptr<octomap::AbstractOcTree>(octomap::AbstractOcTree::read(filename));
    // const auto ocTree = std::dynamic_pointer_cast<octomap::OcTree>(ocTreeBase);
    // octomap_msgs::binaryMapToMsg(*ocTree, octomapMsg);

    auto ocTree = octomap::OcTree(filename);
    octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);

    while (1)
    {
        octomapMsg.header.frame_id = "map";
        octomapPublisher.publish(octomapMsg);
        ROS_INFO("OK. Message length = %d bytes", octomapMsg.data.size());
    }
    ros::spin();

//     auto sendType = SendType::AUTO;
//     auto treeType = TreeType::INVALID_TREE;
//     sendType = SendType::BINARY;
//     sendType = SendType::FULL;
//     treeType = TreeType::BINARY_TREE;
//     treeType = TreeType::OC_TREE;

//     if (treeType == TreeType::OC_TREE) 
//     {
//       const auto ocTreeBase = std::shared_ptr<octomap::AbstractOcTree>(octomap::AbstractOcTree::read(filename));
//       if (sendType == SendType::BINARY) 
//       {
//         const auto ocTree = std::dynamic_pointer_cast<octomap::OcTree>(ocTreeBase);
//         if (ocTree) 
//         {
//           ROS_INFO("Send occupancy tree as binary");
//           octomap_msgs::binaryMapToMsg(*ocTree, octomapMsg);
//         } 
//         else 
//         {
//           throw std::runtime_error("Cannot publish tree");
//         }

//       }
//       else 
//       {
//         ROS_INFO("Send occupancy tree as full map");
//         octomap_msgs::fullMapToMsg(*ocTreeBase, octomapMsg);
//       }

//     } 
//     else 
//     {
//       auto ocTree = octomap::OcTree(filename);
//       if (sendType == SendType::FULL) 
//       {
//         ROS_INFO("Send binary tree as full map");
//         octomap_msgs::fullMapToMsg(ocTree, octomapMsg);
//       } 
//     else
//      {
//         ROS_INFO("Send binary tree as binary");
//         octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);
//       }
//     }

//   while (1)
//   {
//     octomapMsg.header.frame_id = "map";
//     octomapPublisher.publish(octomapMsg);
//     ROS_INFO("OK. Message length = %d bytes", octomapMsg.data.size());
//   }
//   ros::spin();
    return 0;
}
