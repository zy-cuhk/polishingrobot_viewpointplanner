// the function of this cpp is to generate a viewpoint for camera and then obtain the covered octomap and visualize the octoma
// step one: generate a viewpoint 
// step two: obtain the covered octomap 
// step three: visualize the changed octoamp in RVIZ

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <stdio.h>
#include <octomap/math/Utils.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

int main(int argc, char** argv){
// import pcd file 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd.pcd", *cloud) == -1){ 
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
}
// create pcd document octree
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
octomap::point3d Point3dwall(1.1,0,0);    
octomap::Pointcloud pointwall;     
for(int ii=1;ii<321;ii++){
    for(int iii=1;iii<241;iii++){
        Point3dwall.y()= (-0.560027)+(ii*0.003489);
        Point3dwall.z()= (-0.430668)+(iii*0.003574);
        pointwall.push_back(Point3dwall);
    }
}


     
for (int time=0;time<3;time++){
    // change camera pose
    octomap::Pointcloud variablePointwall;     
    octomap::point3d iterator; 
    iterator.x()=0.0;        
    iterator.y()=0.0;  
    iterator.z()=0.430668+time*0.85776;
    
    octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
    octomath::Quaternion Rotation2(0,0,0.0);	
    octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
    variablePointwall=pointwall;		
    variablePointwall.transform(RotandTrans2);

    // add and visualize fov octree
    tree.insertPointCloud(variablePointwall,sensorOrigin);    
    tree.writeBinary("check.bt");  

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
                if (node->getValue()==13){
                    node->setValue(24);
                    known_points_projection++;
                    Continue=false;
                }
            }
        }
    }

    // compute the node which has been changed values
    octomap::point3d iterator1; 
    int coverage_points_num=0;
    int uncoverage_points_num=0;
    for (size_t i = 0; i < cloud->points.size (); ++i){ 
        iterator1.x()=cloud->points[i].x+cloudCentroid[0];
        iterator1.y()=cloud->points[i].y+cloudCentroid[1];
        iterator1.z()=cloud->points[i].z+cloudCentroid[2];
        octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
        if(node1!=NULL){
            if (node1->getValue()==24){
                coverage_points_num++;
            }
            if (node1->getValue()==13){
                uncoverage_points_num++;
            }
        }
    }
    std::cout<<"the points number of wall is: "<<cloud->points.size ()<<std::endl;
    std::cout<<"the coverage points number is: "<<coverage_points_num<<std::endl;
    std::cout<<"the uncoverage points number is: "<<uncoverage_points_num<<std::endl;
    std::cout<<"the points number of camera is: "<<variablePointwall.size()<<std::endl;
    std::cout<<"projected known voxel number is: "<<known_points_projection<<std::endl;
    std::cout<<"---------------------------------"<<std::endl;
}


return 0;
}
