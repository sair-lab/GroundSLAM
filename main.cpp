
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
  std::vector<std::vector<std::string> > frame_lines;
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

    double time_double = dataset.GetTimestamp(i);
    // // for saving 
    // if(map_builder.GetCFPose(new_kcc_pose)){
    //   std::vector<std::string> frame_line;
    //   Eigen::AngleAxisd rotation_vector(new_kcc_pose(2), Eigen::Vector3d(0, 0, 1));
    //   Eigen::Quaterniond q(rotation_vector);
    //   frame_line.emplace_back(std::to_string(time_double));
    //   frame_line.emplace_back(std::to_string(q.w()));
    //   frame_line.emplace_back(std::to_string(q.x()));
    //   frame_line.emplace_back(std::to_string(q.y()));
    //   frame_line.emplace_back(std::to_string(q.z()));
    //   frame_line.emplace_back(std::to_string(new_kcc_pose(0)));
    //   frame_line.emplace_back(std::to_string(new_kcc_pose(1)));
    //   frame_line.emplace_back(std::to_string(0));
    // }

    if((i + 1) >= dataset_length){
      map_builder.CheckAndOptimize();
    }else if(!insert_keyframe){
      continue;
    };  
    std::cout << "Insert a keyframe !" << std::endl;

    // publish pose msgs
    if(map_builder.GetOdomPose(new_odom_pose)){
      visualizer.UpdateOdomPose(new_odom_pose, time_double);
    }
    if(map_builder.GetCFPose(new_kcc_pose)){
      visualizer.UpdateKccPose(new_kcc_pose, time_double);
    }
    if(map_builder.GetFramePoses(frame_poses)){
      visualizer.UpdateFramePose(frame_poses);
    }

    // publish map
    visualizer.UpdateMap(map_builder);

    ros::spinOnce(); 
    loop_rate.sleep(); 
  }

  // saving trajectories 
  std::string saving_root = configs.saving_config.saving_root;
  std::string trajectory_KCC = saving_root + "/KCC_Keyframe.txt";
  std::string trajectory_frame = saving_root + "/optimized_keyframe.txt";

  std::vector<std::vector<std::string> > kcc_keyframe_lines, optimized_keyframe_lines;
  visualizer.GetTrajectoryTxt(kcc_keyframe_lines, Visualizer::TrajectoryType::KCC);
  visualizer.GetTrajectoryTxt(optimized_keyframe_lines, Visualizer::TrajectoryType::Frame);

  WriteTxt(trajectory_KCC, kcc_keyframe_lines, " ");
  WriteTxt(trajectory_frame, optimized_keyframe_lines, " ");
  sleep(5);
};
