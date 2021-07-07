#include "visualization.h"

#include <algorithm>
#include <numeric>
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h> 

#include "utils.h"

void AddNewPoseToPath(
    Eigen::Vector3d& pose, nav_msgs::Path& path, std::string& frame_id){
  ros::Time current_time = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamped; 
  pose_stamped.pose.position.x = pose(0); 
  pose_stamped.pose.position.y = pose(1); 

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose(2)); 
  pose_stamped.pose.orientation.x = q.x; 
  pose_stamped.pose.orientation.y = q.y; 
  pose_stamped.pose.orientation.z = q.z; 
  pose_stamped.pose.orientation.w = q.w; 

  pose_stamped.header.stamp = current_time; 
  pose_stamped.header.frame_id = frame_id; 
  path.poses.push_back(pose_stamped); 
}

void ConvertMapToOccupancyMsgs(OccupancyData& map, nav_msgs::OccupancyGrid& msgs){
  // const int scale = 2;
  // occupancy_map_msgs.info.resolution = occupancy_map_msgs.info.resolution * scale;

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