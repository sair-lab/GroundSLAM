
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "frame.h"
#include "map_stitcher.h"
#include "map_builder.h"
#include "thread_publisher.h"
#include "visualization.h"

#include<typeinfo>
#include <sys/io.h>
#include <sys/dir.h>
#include <time.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "build_map");
  ros::start(); 

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config; 
  Dataset dataset(dataset_config.dataroot, dataset_config.image_dir_name); 

  MapBuilder map_builder(configs); 
  Visualizer visualizer(configs.visualization_config); 
  Aligned<std::vector, Eigen::Vector3d> frame_poses;
  std::vector<double> timestamps;
  Eigen::Vector3d new_kcc_pose; 
  ros::Rate loop_rate(50); 
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
    double time_double = dataset.GetTimestamp(i);
    visualizer.PublishImage(image, time_double);
    auto t1 = std::chrono::high_resolution_clock::now();
    bool insert_keyframe = map_builder.AddNewInput(image, time_double); // error if image size is wrong
    auto t2 = std::chrono::high_resolution_clock::now();
    auto compute_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
    cout << "processing for one frame is " << compute_time << "ms" << endl;

    
    if((i + 1) >= dataset_length){
      map_builder.CheckAndOptimize();
    }else if(!insert_keyframe){
      continue;
    };  
    std::cout << "Insert a keyframe !" << std::endl;

    // publish pose msgs
    if(map_builder.GetCFPose(new_kcc_pose)){    
      visualizer.UpdateKccPose(new_kcc_pose, time_double);
    }
    if(map_builder.GetFramePoses(frame_poses, timestamps)){
      visualizer.UpdateFramePose(frame_poses, timestamps);
    }

    visualizer.UpdateMap(map_builder);
    ros::spinOnce(); 
    loop_rate.sleep(); 
  }

  // save trajectories 
  std::string saving_root = configs.saving_config.saving_root;
  MakeDir(saving_root);
  std::string trajectory_KCC = saving_root + "/KCC_Keyframe.txt";
  std::string trajectory_frame = saving_root + "/optimized_keyframe.txt";
  std::vector<std::vector<std::string> > kcc_keyframe_lines, optimized_keyframe_lines;
  visualizer.GetTrajectoryTxt(kcc_keyframe_lines, Visualizer::TrajectoryType::KCC);
  visualizer.GetTrajectoryTxt(optimized_keyframe_lines, Visualizer::TrajectoryType::Frame);

  WriteTxt(trajectory_KCC, kcc_keyframe_lines, " ");
  WriteTxt(trajectory_frame, optimized_keyframe_lines, " ");
};
