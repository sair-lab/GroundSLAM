#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <Eigen/Core>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include "read_configs.h"
#include "map_stitcher.h"
#include "map_builder.h"

class Visualizer{
public:
  enum class TrajectoryType {
    Frame = 0,
    KCC = 1,
    Odom = 2,
  };

  Visualizer(VisualizationConfig& config);
  void AddNewPoseToPath(
    Eigen::Vector3d& pose, nav_msgs::Path& path, std::string& frame_id);
  void UpdateOdomPose(Eigen::Vector3d& pose);
  void UpdateKccPose(Eigen::Vector3d& pose);
  void UpdateFramePose(Aligned<std::vector, Eigen::Vector3d>& frame_poses);
  void ConvertMapToOccupancyMsgs(OccupancyData& map, nav_msgs::OccupancyGrid& msgs);
  void UpdateMap(MapBuilder& map_builder);

  void GetTrajectoryTxt(std::vector<std::vector<std::string> >& lines, TrajectoryType trajectory_type);

private:
  ros::NodeHandle nh;
  std::string frame_id;
  ros::Publisher odom_pose_pub;
  ros::Publisher kcc_pose_pub;
  ros::Publisher frame_pose_pub;
  ros::Publisher map_pub;

  nav_msgs::Path odom_pose_msgs;
  nav_msgs::Path kcc_pose_msgs;
  nav_msgs::Path frame_pose_msgs;
  nav_msgs::OccupancyGrid occupancy_map_msgs;
};

#endif  // VISUALIZATION_H_