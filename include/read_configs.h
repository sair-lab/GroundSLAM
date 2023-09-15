#ifndef READ_CONFIGS_H_
#define READ_CONFIGS_H_

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "utils.h"

struct DatasetConfig{
  std::string dataroot;
  std::string image_dir_name;
  std::string camera_file;
};

struct CFConfig{
  int width;
  int height;
  float lambda;
  int kernel;
  float sigma;
  float offset;
  int power;
  int rotation_divisor;
  int rotation_channel;
};

struct KeyframeSelectionConfig{
  double max_distance;
  double max_angle;
  double lower_response_thr;
  double upper_response_thr;
};

struct MapConfig{
  double grid_scale;
};

struct LoopClosureConfig{
  bool to_find_loop;
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
  std::string kcc_pose_topic;
  std::string frame_pose_topic;
  std::string map_topic;
  std::string image_topic;
};

struct SavingConfig{
  std::string saving_root;
  bool save_pose;
};

struct Configs{
  DatasetConfig dataset_config;
  CFConfig cf_config;
  KeyframeSelectionConfig keyframe_selection_config;
  MapConfig map_config;
  LoopClosureConfig loop_closure_config;
  MapStitcherConfig map_stitcher_config;
  VisualizationConfig visualization_config;
  SavingConfig saving_config;

  Configs(const std::string& config_file){
    if(!FileExists(config_file)){
      std::cout << "config file: " << config_file << " doesn't exist" << std::endl;
      return;
    }

    YAML::Node file_node = YAML::LoadFile(config_file);
    YAML::Node dataset_node = file_node["dataset"];
    dataset_config.dataroot = dataset_node["dataroot"].as<std::string>();
    dataset_config.image_dir_name = dataset_node["image_dir_name"].as<std::string>();
    dataset_config.camera_file = dataset_node["camera_config"].as<std::string>();

    YAML::Node cf_node = file_node["correlation_flow"];
    cf_config.width = cf_node["width"].as<int>();
    cf_config.height = cf_node["height"].as<int>();
    cf_config.lambda = cf_node["lambda"].as<float>();
    cf_config.rotation_divisor = cf_node["rotation_divisor"].as<int>();
    cf_config.rotation_channel = cf_node["rotation_channel"].as<int>();
    cf_config.kernel = cf_node["kernel"].as<int>();
    cf_config.offset = cf_node["polynomial"]["offset"].as<float>();
    cf_config.power = cf_node["polynomial"]["power"].as<int>();
    cf_config.sigma = cf_node["gaussian"]["sigma"].as<float>();

    YAML::Node kfs_node = file_node["keyframe_selection"];
    keyframe_selection_config.max_distance = kfs_node["max_distance"].as<double>();
    keyframe_selection_config.max_angle = kfs_node["max_angle"].as<double>();
    keyframe_selection_config.lower_response_thr = kfs_node["lower_response_thr"].as<double>();
    keyframe_selection_config.upper_response_thr = kfs_node["upper_response_thr"].as<double>();

    YAML::Node map_node = file_node["map"];
    map_config.grid_scale = map_node["grid_scale"].as<double>();

    YAML::Node loop_closure_node = file_node["loop_closure"];
    loop_closure_config.to_find_loop = 
        loop_closure_node["to_find_loop"].as<bool>();
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
    visualization_config.kcc_pose_topic = visualization_node["topic"]["kcc_pose"].as<std::string>();
    visualization_config.frame_pose_topic = visualization_node["topic"]["frame_pose"].as<std::string>();
    visualization_config.map_topic = visualization_node["topic"]["map"].as<std::string>();
    visualization_config.image_topic = visualization_node["topic"]["image"].as<std::string>();

    YAML::Node saving_node = file_node["saving"];
    saving_config.saving_root = saving_node["saving_root"].as<std::string>();
    saving_config.save_pose = saving_node["save_pose"].as<bool>();
  }
};



#endif  // READ_CONFIGS_H_