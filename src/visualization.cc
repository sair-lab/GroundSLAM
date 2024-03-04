#include "visualization.h"

#include <algorithm>
#include <numeric>
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 
#include<time.h>

#include "utils.h"

#include <iostream>

using namespace std;

Visualizer::Visualizer(VisualizationConfig& config): frame_id(config.frame_id){
  // odom_pose_pub = nh.advertise<nav_msgs::Path>(config.odom_pose_topic, 10);
  kcc_pose_pub = nh.advertise<nav_msgs::Path>(config.kcc_pose_topic, 10);
  frame_pose_pub = nh.advertise<nav_msgs::Path>(config.frame_pose_topic, 10);
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>(config.map_topic, 1);
  image_pub = nh.advertise<sensor_msgs::Image>(config.image_topic, 1);

  ros::Time current_time = ros::Time::now();
  odom_pose_msgs.header.stamp = current_time; 
	odom_pose_msgs.header.frame_id = frame_id; 
  kcc_pose_msgs.header.stamp = current_time; 
	kcc_pose_msgs.header.frame_id = frame_id; 
  frame_pose_msgs.header.stamp = current_time; 
	frame_pose_msgs.header.frame_id = frame_id; 

  occupancy_map_msgs.header.stamp = current_time;
  occupancy_map_msgs.header.frame_id = frame_id;
}

void Visualizer::AddNewPoseToPath(
    Eigen::Vector3d& pose, double time_double, nav_msgs::Path& path, std::string& id){
  ros::Time current_time = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamped; 
  pose_stamped.pose.position.x = pose(0); 
  pose_stamped.pose.position.y = pose(1); 
  pose_stamped.pose.position.z = 0; 

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose(2)); 
  pose_stamped.pose.orientation.x = q.x; 
  pose_stamped.pose.orientation.y = q.y; 
  pose_stamped.pose.orientation.z = q.z; 
  pose_stamped.pose.orientation.w = q.w; 

  if(time_double < 0){
    pose_stamped.header.stamp = current_time; 
  }else{
    pose_stamped.header.stamp = ros::Time().fromSec(time_double);
    int64_t sec = static_cast<int64_t>(pose_stamped.header.stamp.sec);
    int64_t nsec = static_cast<int64_t>(pose_stamped.header.stamp.nsec);
    std::string s_time = std::to_string(sec) + "." + std::to_string(nsec);
  }
  
  pose_stamped.header.frame_id = id; 
  path.poses.push_back(pose_stamped); 
}

void Visualizer::UpdateOdomPose(Eigen::Vector3d& pose, double time_double){
  AddNewPoseToPath(pose, time_double, odom_pose_msgs, frame_id);
  // odom_pose_pub.publish(odom_pose_msgs); 
}

void Visualizer::UpdateKccPose(Eigen::Vector3d& pose, double time_double){
  AddNewPoseToPath(pose, time_double, kcc_pose_msgs, frame_id);
  kcc_pose_pub.publish(kcc_pose_msgs); 
}

void Visualizer::UpdateFramePose(Aligned<std::vector, Eigen::Vector3d>& frame_poses, std::vector<double>& timestamps){
  frame_pose_msgs.poses.clear();
  for(size_t i = 0; i < frame_poses.size(); i++){
    AddNewPoseToPath(frame_poses[i], timestamps[i], frame_pose_msgs, frame_id);
  }
  frame_pose_pub.publish(frame_pose_msgs); 

}

void Visualizer::ConvertMapToOccupancyMsgs(
    OccupancyData& map, nav_msgs::OccupancyGrid& msgs){
  if(map.size() < 1) return;

  // fill in metadata
  int max_x = std::numeric_limits<int>::min();
  int max_y = std::numeric_limits<int>::min();
  int min_x = std::numeric_limits<int>::max();
  int min_y = std::numeric_limits<int>::max();

  for(auto& kv : map){
    int x = kv.first.x;
    int y = kv.first.y;
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
  }

  if((min_x > max_x) || (min_y > max_y)) return;

  int size = map.begin()->second.size;
  msgs.info.width = (max_x - min_x + 1) * size;
  msgs.info.height = (max_y - min_y + 1) * size;

  msgs.info.origin.position.x = (min_x * size);
  msgs.info.origin.position.y = (min_y * size);
  msgs.info.origin.position.z = 0;

  // fill in map_data
  int num_grids = msgs.info.width * msgs.info.height;
  std::vector<int8_t> data(num_grids, -1);

  for(auto& kv : map){
    for(int i = 0; i < kv.second.size; i++){
      for(int j = 0; j < kv.second.size; j++){
        if(kv.second.weight(i, j) < 1) continue;

        int x = (kv.first.x - min_x) * kv.second.size + j;
        int y = (kv.first.y - min_y) * kv.second.size + i;
        int idx = y * msgs.info.width + x;
        int pixel = static_cast<int>(kv.second.data(i, j));

        data[idx] = 100 - static_cast<int8_t>(pixel);
      }
    }
  }
  msgs.data = data;
}

void Visualizer::UpdateMap(MapBuilder& map_builder){
  occupancy_map_msgs.info.resolution = map_builder.GetMapResolution();
  OccupancyData map_data = map_builder.GetMapData();
  if(map_data.size() < 1) return;
  ConvertMapToOccupancyMsgs(map_data, occupancy_map_msgs);

  Eigen::Matrix<double, 7, 1> real_origin;
  Eigen::Vector3d pixel_origin;
  pixel_origin << occupancy_map_msgs.info.origin.position.x, occupancy_map_msgs.info.origin.position.y, occupancy_map_msgs.info.origin.position.z;
  map_builder.GetOccupancyMapOrigin(pixel_origin, real_origin);
  occupancy_map_msgs.info.origin.orientation.w = real_origin(0, 0);
  occupancy_map_msgs.info.origin.orientation.x = real_origin(1, 0);
  occupancy_map_msgs.info.origin.orientation.y = real_origin(2, 0);
  occupancy_map_msgs.info.origin.orientation.z = real_origin(3, 0);
  occupancy_map_msgs.info.origin.position.x = real_origin(4, 0);
  occupancy_map_msgs.info.origin.position.y = real_origin(5, 0);
  occupancy_map_msgs.info.origin.position.z = real_origin(6, 0);

  map_pub.publish(occupancy_map_msgs);
}

void Visualizer::PublishImage(cv::Mat& image, double time_double){
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
  if(time_double > 0){
    image_msg->header.stamp = ros::Time().fromSec(time_double);
  }
  image_pub.publish(image_msg);
}

void Visualizer::GetTrajectoryTxt(
    std::vector<std::vector<std::string> >& lines, TrajectoryType trajectory_type){
  nav_msgs::Path* path_ptr;
  switch(trajectory_type){
    case TrajectoryType::Frame:
      path_ptr = &frame_pose_msgs;
      break;
    case TrajectoryType::KCC:
      path_ptr = &kcc_pose_msgs;
      break;
    case TrajectoryType::Odom:
      path_ptr = &odom_pose_msgs;
      break;
    default:
      std::cout << "please select trajectory_type from Frame, KCC and Odom !" << std::endl; 
      return;
  }

  for(geometry_msgs::PoseStamped& pose_stamped : (*path_ptr).poses){
    std::vector<std::string> line;
    int64_t sec = static_cast<int64_t>(pose_stamped.header.stamp.sec);
    int64_t nsec = static_cast<int64_t>(pose_stamped.header.stamp.nsec);
    // std::string s_time = std::to_string(sec) + "." + std::to_string(nsec);

    double sec_double = static_cast<double>(sec);
    double nsec_double = static_cast<double>(nsec) / 1e9;
    double time_double = sec_double + nsec_double;
    std::string s_time = std::to_string(time_double);

    line.emplace_back(s_time);
    line.emplace_back(std::to_string(pose_stamped.pose.position.x));
    line.emplace_back(std::to_string(pose_stamped.pose.position.y));
    line.emplace_back(std::to_string(pose_stamped.pose.position.z));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.x));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.y));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.z));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.w));
    lines.push_back(line);
  }
}


