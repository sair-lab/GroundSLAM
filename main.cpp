
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


#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "map_stitcher.h"
#include "map_builder.h"
#include "thread_publisher.h"
#include "visualization.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "build_map");
  ros::start();
  google::InitGoogleLogging(argv[0]);

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config;
  Dataset dataset(dataset_config.dataroot);

  const bool OdomPoseIsAvailable = dataset.PoseIsAvailable();
  MapBuilder map_builder(configs, OdomPoseIsAvailable);

  Visualizer visualizer(configs.visualization_config);

  Aligned<std::vector, Eigen::Vector3d> frame_poses;
  Eigen::Vector3d new_odom_pose, new_kcc_pose;

  ros::Rate loop_rate(100);
  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length; ++i){
    if(!ros::ok()) break; 
    std::cout << i << std::endl;

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
    if((i + 1) >= dataset_length){
      map_builder.CheckAndOptimize();
    }else if(!insert_keyframe){
      continue;
    };  
    std::cout << "Insert a keyframe !" << std::endl;

    // publish pose msgs
    if(map_builder.GetOdomPose(new_odom_pose)){
      visualizer.UpdateOdomPose(new_odom_pose);
    }
    if(map_builder.GetCFPose(new_kcc_pose)){
      visualizer.UpdateKccPose(new_kcc_pose);
    }
    if(map_builder.GetFramePoses(frame_poses)){
      visualizer.UpdateFramePose(frame_poses);
    }

    // publish map
    visualizer.UpdateMap(map_builder);

    ros::spinOnce(); 
    loop_rate.sleep(); 
  }
  
  sleep(5);
};
