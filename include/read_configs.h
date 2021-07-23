#ifndef READ_CONFIGS_H_
#define READ_CONFIGS_H_

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "utils.h"

struct DatasetConfig{
  std::string dataroot;
  std::string camera_file;
};

struct CFConfig{
  int width;
  int height;
  float sigma;
  float lambda;
  int rotation_divisor;
  int rotation_channel;
};

struct KeyframeSelectionConfig{
  double min_distance;
  double max_distance;
  double min_angle;
  double min_position_response;
  double min_angle_response;
};

struct MapConfig{
  double grid_scale;
};

struct LoopClosureConfig{
  double position_response_thr;
  double angle_response_thr;
  int frame_gap_thr;
  double distance_thr;
};

struct MapStitcherConfig{
  bool stitch_map;
  int cell_size;
};

struct VisualizationConfig{
  std::string frame_id;
  std::string odom_pose_topic;
  std::string kcc_pose_topic;
  std::string frame_pose_topic;
  std::string map_topic;
};

struct Configs{
  DatasetConfig dataset_config;
  CFConfig cf_config;
  KeyframeSelectionConfig keyframe_selection_config;
  MapConfig map_config;
  LoopClosureConfig loop_closure_config;
  MapStitcherConfig map_stitcher_config;
  VisualizationConfig visualization_config;

  Configs(const std::string& config_file){
    if(!FileExists(config_file)){
      std::cout << "config file: " << config_file << " doesn't exist" << std::endl;
      return;
    }

    YAML::Node file_node = YAML::LoadFile(config_file);
    YAML::Node dataset_node = file_node["dataset"];
    dataset_config.dataroot = dataset_node["dataroot"].as<std::string>();
    dataset_config.camera_file = dataset_node["camera_config"].as<std::string>();

    YAML::Node cf_node = file_node["correlation_flow"];
    cf_config.width = cf_node["width"].as<int>();
    cf_config.height = cf_node["height"].as<int>();
    cf_config.sigma = cf_node["sigma"].as<float>();
    cf_config.lambda = cf_node["lambda"].as<float>();
    cf_config.rotation_divisor = cf_node["rotation_divisor"].as<int>();
    cf_config.rotation_channel = cf_node["rotation_channel"].as<int>();

    YAML::Node kfs_node = file_node["keyframe_selection"];
    keyframe_selection_config.min_distance = kfs_node["min_distance"].as<double>();
    keyframe_selection_config.max_distance = kfs_node["max_distance"].as<double>();
    keyframe_selection_config.min_angle = kfs_node["min_angle"].as<double>();
    keyframe_selection_config.min_position_response = kfs_node["min_position_response"].as<double>();
    keyframe_selection_config.min_angle_response = kfs_node["min_angle_response"].as<double>();

    YAML::Node map_node = file_node["map"];
    map_config.grid_scale = map_node["grid_scale"].as<double>();

    YAML::Node loop_closure_node = file_node["loop_closure"];
    loop_closure_config.position_response_thr = 
        loop_closure_node["position_response_thr"].as<double>();
    loop_closure_config.angle_response_thr = 
        loop_closure_node["angle_response_thr"].as<double>();
    loop_closure_config.frame_gap_thr = 
        loop_closure_node["frame_gap_thr"].as<int>();
    loop_closure_config.distance_thr = 
        loop_closure_node["distance_thr"].as<double>();

    YAML::Node map_stitcher_node = file_node["map_sticther"];
    map_stitcher_config.stitch_map = map_stitcher_node["stitch_map"].as<bool>();
    map_stitcher_config.cell_size = map_stitcher_node["cell_size"].as<int>();

    YAML::Node visualization_node = file_node["visualization"];
    visualization_config.frame_id = visualization_node["frame_id"].as<std::string>();
    visualization_config.odom_pose_topic = visualization_node["topic"]["odom_pose"].as<std::string>();
    visualization_config.kcc_pose_topic = visualization_node["topic"]["kcc_pose"].as<std::string>();
    visualization_config.frame_pose_topic = visualization_node["topic"]["frame_pose"].as<std::string>();
    visualization_config.map_topic = visualization_node["topic"]["map"].as<std::string>();
  }
};



#endif  // READ_CONFIGS_H_