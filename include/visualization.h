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

#include "map_stitcher.h"

void AddNewPoseToPath(
    Eigen::Vector3d& pose, nav_msgs::Path& path, std::string& frame_id);

void ConvertMapToOccupancyMsgs(OccupancyMap& map, nav_msgs::OccupancyGrid& msgs);

#endif  // VISUALIZATION_H_