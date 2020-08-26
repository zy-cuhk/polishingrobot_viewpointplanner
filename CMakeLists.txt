cmake_minimum_required(VERSION 2.8.3)
project(polishingrobot_viewpointplanner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  octomap_msgs
	rospy
	std_msgs
	rostime 
	sensor_msgs 
	message_filters 
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
    octomap_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(collision_check src/moveit_collision_check_v2.cpp)
target_link_libraries(collision_check ${catkin_LIBRARIES})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(publish_pointcloud src/publish_pointcloud.cpp)
target_link_libraries(publish_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS publish_pointcloud RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(viewpoint_planning_using_octomap src/viewpoint_planning_using_octomap.cpp)
target_link_libraries(viewpoint_planning_using_octomap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(publish_octomap src/publish_octomap.cpp)
add_dependencies(publish_octomap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_octomap ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS publish_octomap DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



