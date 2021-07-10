
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <unistd.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "map_stitcher.h"
#include "map_builder.h"
#include "thread_publisher.h"
#include "visualization.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "build_map");
  google::InitGoogleLogging(argv[0]);

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config;
  Dataset dataset(dataset_config.dataroot);

  const bool OdomPoseIsAvailable = dataset.PoseIsAvailable();
  MapBuilder map_builder(configs, OdomPoseIsAvailable);

  // ros publisher
  ros::NodeHandle nh;
  ros::Publisher odom_pose_pub = nh.advertise<nav_msgs::Path>("/kcc_slam/odom_pose", 10);
  ros::Publisher kcc_pose_pub = nh.advertise<nav_msgs::Path>("/kcc_slam/kcc_pose", 10);
  ros::Publisher frame_pose_pub = nh.advertise<nav_msgs::Path>("/kcc_slam/frame_pose", 10);
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/kcc_slam/occupancy_map", 1);

  ros::Time current_time = ros::Time::now();
  std::string frame_id = "map";

  nav_msgs::Path odom_pose_msgs, kcc_pose_msga, frame_pose_msgs;
  nav_msgs::OccupancyGrid occupancy_map_msgs;

  odom_pose_msgs.header.stamp = current_time; 
	odom_pose_msgs.header.frame_id = frame_id; 
  kcc_pose_msga.header.stamp = current_time; 
	kcc_pose_msga.header.frame_id = frame_id; 
  frame_pose_msgs.header.stamp = current_time; 
	frame_pose_msgs.header.frame_id = frame_id; 

  occupancy_map_msgs.header.stamp = current_time;
  occupancy_map_msgs.header.frame_id = frame_id;

  Aligned<std::vector, Eigen::Vector3d> frame_poses;
  Eigen::Vector3d new_odom_pose, new_kcc_pose;

  ros::Rate loop_rate(50);
  size_t dataset_length = dataset.GetDatasetLength();
  int skip = 1;
  for(size_t i = 0; i < dataset_length; ++i){
    if(!ros::ok()) break; 

    std::cout << i << std::endl;
    if (i%skip != 0){
      continue;
    }
    cv::Mat image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }

    Eigen::Vector3d pose;
    if(OdomPoseIsAvailable){
      dataset.GetPose(pose, i);
    }

    bool insert_keyframe = map_builder.AddNewInput(image, pose);

    if((i + skip) >= dataset_length){
      map_builder.CheckAndOptimize();
    }else if(!insert_keyframe){
      continue;
    };  
    std::cout << "Insert a keyframe !" << std::endl;

    // publish pose msgs
    if(map_builder.GetOdomPose(new_odom_pose)){
      AddNewPoseToPath(new_odom_pose, odom_pose_msgs, frame_id);
      odom_pose_pub.publish(odom_pose_msgs); 
    }
    if(map_builder.GetCFPose(new_kcc_pose)){
      AddNewPoseToPath(new_kcc_pose, kcc_pose_msga, frame_id);
      kcc_pose_pub.publish(kcc_pose_msga); 
    }
    if(map_builder.GetFramePoses(frame_poses)){
      frame_pose_msgs.poses.clear();
      for(auto pose : frame_poses){
        AddNewPoseToPath(pose, frame_pose_msgs, frame_id);
      }
      frame_pose_pub.publish(frame_pose_msgs); 
    }

    // std::cout << "new_odom_pose = " << new_odom_pose.transpose() << std::endl;
    // std::cout << "new_kcc_pose = " << new_kcc_pose.transpose() << std::endl;
    // std::cout << "new_frame_pose = " << frame_poses[(frame_poses.size()-1)].transpose() << std::endl;


    // publish map
    occupancy_map_msgs.info.resolution = map_builder.GetMapResolution();
    OccupancyData map_data = map_builder.GetMapData();
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

    ros::spinOnce(); 
    loop_rate.sleep(); 
  }
  
  sleep(5);
};
