#include "map_builder.h"

#include <set>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "edge.h"
#include "utils.h"
#include "optimization_2d/types.h"
#include "optimization_2d/pose_graph_2d.h"

MapBuilder::MapBuilder(Configs& configs, const bool odom_is_available):
    OdomPoseIsAvailable(odom_is_available), _init(false), _frame_id(0), _edge_id(0){
  _camera = std::shared_ptr<Camera>(new Camera(configs.dataset_config.camera_file));
  _correlation_flow = std::shared_ptr<CorrelationFlow>(new CorrelationFlow(configs.cf_config));
  _map = std::shared_ptr<Map>(new Map(configs.map_config));

  _loop_closure = std::shared_ptr<LoopClosure>(
      new LoopClosure(configs.loop_closure_config, _correlation_flow, _map));

  _map_stitcher = std::shared_ptr<MapStitcher>(new MapStitcher(_camera));
}

void MapBuilder::AddNewInput(cv::Mat& image, Eigen::Vector3d& odom_pose){
  if(OdomPoseIsAvailable){
    _current_odom_pose = odom_pose;
  }else{
    _current_odom_pose << 0.0, 0.0, 0.0;
  }
  cv::Mat undistort_image;
  _camera->UndistortImage(image, undistort_image);

  ComputeFFTResult(undistort_image);
  ConstructFrame();
  
  if(!_init){
    Initialize();
    _map_stitcher->InsertFrame(_current_frame, undistort_image);
    UpdateIntermedium();
    return;
  }
  
  Eigen::Vector3d relative_pose;
  bool good_tracking = Tracking(relative_pose);
  if(good_tracking){
    _current_cf_pose = ComputeAbsolutePose(_last_cf_pose, relative_pose);
    _camera->ConvertImagePlanePoseToCamera(_current_cf_pose, _current_cf_real_pose);
    Eigen::Vector3d relative_cf_real_pose = ComputeRelativePose(_last_cf_real_pose, _current_cf_real_pose);
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    AddCFEdgeToMap(relative_cf_real_pose, _last_frame->GetFrameId(), 
    _current_frame->GetFrameId(), _edge_id++, info);
  }else{
    if(!OdomPoseIsAvailable) return;
    _camera->ConvertRobotPoseToCamera(_current_cf_real_pose, _current_odom_pose);
    _camera->ConvertCameraPoseToImagePlane(_current_cf_pose, _current_cf_real_pose);
  }
  
  AddOdomEdgeToMap();
  UpdateCurrentPose();
  SetCurrentFramePose();
  _map->AddFrame(_current_frame);
  SetFrameDistance();
  _map_stitcher->InsertFrame(_current_frame, undistort_image);

  bool loop_found = FindLoopClosure();
  if(!loop_found){
    if(_loop_matches.size() >= 2){
      AddLoopEdges();
      OptimizeMap();
      _map_stitcher->RecomputeOccupancy();
    }
    _loop_matches.clear();
  }

  UpdateIntermedium();
}

void MapBuilder::ComputeFFTResult(cv::Mat& image){
  ConvertMatToNormalizedArray(image, _image_array);
  _correlation_flow->ComputeIntermedium(_image_array, _fft_result, _fft_polar);
}

void MapBuilder::ConstructFrame(){
  _current_frame = std::shared_ptr<Frame>(
      new Frame(_frame_id++, _image_array, _fft_result, _fft_polar));
}

void MapBuilder::SetCurrentFramePose(){
  _current_frame->SetPose(_current_pose);
}

bool MapBuilder::Initialize(){
  _current_cf_pose << 0.0, 0.0, 0.0;
  _camera->ConvertImagePlanePoseToCamera(_current_cf_pose, _current_cf_real_pose);
  _camera->ConvertCameraPoseToRobot(_current_cf_real_pose, _current_pose);
  _baseframe_odom_pose = _current_odom_pose;
  SetCurrentFramePose();
  _map->AddFrame(_current_frame);
  _distance = 0;
  _map->SetFrameDistance(_current_frame, _distance);
  _init = true;
  _last_lost = false;
  return true;
}

void MapBuilder::UpdateIntermedium(){
  _last_frame = _current_frame;
  _last_cf_pose = _current_cf_pose;
  _last_cf_real_pose = _current_cf_real_pose;
  _last_odom_pose = _current_odom_pose;
  _last_pose = _current_pose;
  _last_fft_result = _fft_result;
  _last_fft_polar = _fft_polar;
}

void MapBuilder::UpdateCurrentPose(){
  Eigen::Vector3d last_robot_pose_from_cf, current_robot_pose_from_cf;
  _camera->ConvertImagePlanePoseToRobot(_last_cf_pose, last_robot_pose_from_cf);
  _camera->ConvertImagePlanePoseToRobot(_current_cf_pose, current_robot_pose_from_cf);
  Eigen::Vector3d relativate_pose_from_cf = ComputeRelativePose(last_robot_pose_from_cf, current_robot_pose_from_cf);
  Eigen::Vector3d relative_odom_pose = ComputeRelativePose(_last_odom_pose, _current_odom_pose);

  Eigen::Vector3d relative_pose = relativate_pose_from_cf;
  // relative_pose(2) = relative_odom_pose(2);
  _current_pose = ComputeAbsolutePose(_last_pose, relative_pose);
  // _camera->ConvertImagePlanePoseToRobot(_current_cf_pose, _current_pose);
}

bool MapBuilder::Tracking(Eigen::Vector3d& relative_pose){
  Eigen::Vector3d response = _correlation_flow->ComputePose(
      _last_fft_result, _image_array, _last_fft_polar, _fft_polar, relative_pose);
  std::cout << "cf_edge response = " << response.transpose() << std::endl;
  return true;
}

void MapBuilder::AddCFEdgeToMap(
    Eigen::Vector3d& relative_pose, int from, int to, int edge_id, Eigen::Matrix3d& info){
  EdgePtr cf_edge = std::make_shared<Edge>();
  cf_edge->_edge_id = edge_id;
  cf_edge->_type = Edge::Type::KCC;
  cf_edge->_from = from;
  cf_edge->_to = to;
  cf_edge->_T = relative_pose;
  cf_edge->_information = info;
  // std::cout << "cf_edge relative_pose = " << relative_pose.transpose() << std::endl;
  _map->AddEdge(cf_edge);
}

void MapBuilder::AddOdomEdgeToMap(){
  if(!OdomPoseIsAvailable) return;

  Eigen::Vector3d relative_odom_pose = 
      ComputeRelativePose(_last_pose, _current_pose);

  EdgePtr odom_edge = std::make_shared<Edge>();
  odom_edge->_edge_id = _edge_id++;
  odom_edge->_type = Edge::Type::Odom;
  odom_edge->_from = _last_frame->GetFrameId();
  odom_edge->_to = _current_frame->GetFrameId();
  odom_edge->_T = relative_odom_pose;
  odom_edge->_information = Eigen::Matrix3d::Identity();
  std::cout << "odom_edge relative_pose = " << relative_odom_pose.transpose() << std::endl;
  _map->AddEdge(odom_edge);
}

void MapBuilder::SetFrameDistance(){
  Eigen::Vector3d rlt_pose = _current_pose - _last_pose;
  Eigen::Vector3d rlt_camera_pose;
  if(OdomPoseIsAvailable){
    _camera->ConvertRobotPoseToCamera(rlt_camera_pose, rlt_pose);
  }else{
    rlt_camera_pose = rlt_pose;
  }

  double d = sqrt(rlt_camera_pose.head(2).dot(rlt_camera_pose.head(2)));
  _distance += d;
  _map->SetFrameDistance(_current_frame, _distance);
}

bool MapBuilder::FindLoopClosure(){
  LoopClosureResult loop_closure_result = 
      _loop_closure->FindLoopClosure(_image_array, _current_frame, _current_pose);
  if(loop_closure_result.found){
    _loop_matches.emplace_back(loop_closure_result);
  }
  return loop_closure_result.found;
}

void MapBuilder::AddLoopEdges(){
  for(auto loop_match : _loop_matches){
    int edge_id = _edge_id++;
    int from = loop_match.loop_frame->GetFrameId();
    int to = loop_match.current_frame->GetFrameId();
    Eigen::Vector3d relative_pose = loop_match.relative_pose;
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    AddCFEdgeToMap(relative_pose, from, to, edge_id, info);
  }
}

bool MapBuilder::OptimizeMap(){
  std::map<int, ceres::optimization_2d::Pose2d> poses;
  std::vector<ceres::optimization_2d::Constraint2d> constraints;

  std::vector<FramePtr> frames;
  std::set<int> frame_ids;
  std::vector<EdgePtr> edges;
  _map->GetAllFrames(frames);
  _map->GetAllEdges(edges);

  for(FramePtr& frame : frames){
    int frame_id = frame->GetFrameId();
    Eigen::Vector3d pose_vec;
    frame->GetPose(pose_vec);

    frame_ids.insert(frame_id);
    ceres::optimization_2d::Pose2d pose_2d;
    pose_2d.x = pose_vec(0);
    pose_2d.y = pose_vec(1);
    pose_2d.yaw_radians = pose_vec(2);
    poses.insert(std::pair<int, ceres::optimization_2d::Pose2d>(frame_id, pose_2d));
  }

  std::vector<ceres::optimization_2d::ScaleData> scale_data;
  ceres::optimization_2d::ScaleData cf_scale;
  cf_scale.scale = _camera->GetImageHeight();
  if(OdomPoseIsAvailable && (!_camera->HeightIsAccurate())){
    cf_scale.fixed = false;
  }else{
    cf_scale.fixed = true;
  }
  scale_data.emplace_back(cf_scale);

  ceres::optimization_2d::ScaleData odom_scale(1.0, true);
  scale_data.emplace_back(odom_scale);

  std::vector<int> scale_data_idx;
  for(EdgePtr& edge : edges){
    int from = edge->_from;
    int to = edge->_to;
    if((frame_ids.count(from) < 1) || (frame_ids.count(to) < 1)){
      continue;
    }

    if(edge->_type == Edge::Type::Odom){
      scale_data_idx.emplace_back(0);
    }else if(edge->_type == Edge::Type::KCC || edge->_type == Edge::Type::Loop){
      scale_data_idx.emplace_back(1);
    }else{
      continue;
    }

    ceres::optimization_2d::Constraint2d constraint;
    constraint.id_begin = from;
    constraint.id_end = to;
    constraint.x = edge->_T(0);
    constraint.y = edge->_T(1);
    constraint.yaw_radians = edge->_T(2);
    constraints.emplace_back(constraint);
  }

  ceres::Problem problem;
  ceres::optimization_2d::BuildOptimizationProblem(constraints, scale_data, scale_data_idx, &poses, &problem);
  CHECK(ceres::optimization_2d::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  // copy back frame pose
  AlignedMap<int, Eigen::Vector3d> frame_poses;
  for(auto kv : poses){
    Eigen::Vector3d frame_pose;
    frame_pose << kv.second.x, kv.second.y, kv.second.yaw_radians;
    frame_poses[kv.first] = frame_pose;
  }

  return true;
}

// for visualization
bool MapBuilder::GetOdomPose(Eigen::Vector3d& pose){
  if(!OdomPoseIsAvailable) return false;
  pose = ComputeRelativePose(_baseframe_odom_pose, _current_odom_pose);
  return true;
}

bool MapBuilder::GetCFPose(Eigen::Vector3d& pose){
  Eigen::Vector3d cf_base_pose(0.0, 0.0, 0.0);
  Eigen::Vector3d robot_base_pose;
  _camera->ConvertImagePlanePoseToRobot(cf_base_pose, robot_base_pose);

  Eigen::Vector3d robot_current_pose;
  _camera->ConvertImagePlanePoseToRobot(_current_cf_pose, robot_current_pose);
  pose = ComputeRelativePose(robot_base_pose, robot_current_pose);
  return true;
}

bool MapBuilder::GetFramePoses(Aligned<std::vector, Eigen::Vector3d>& poses){
  poses.clear();
  std::vector<FramePtr> frames;
  _map->GetAllFrames(frames);

  for(auto frame : frames){
    Eigen::Vector3d pose;
    frame->GetPose(pose);
    poses.emplace_back(pose);
  }

  return true;
}

bool MapBuilder::GetOccupancyMapOrigin(Eigen::Matrix<double, 7, 1>& origin){
  Eigen::Matrix3d Rbc;
  _camera->GetExtrinsics(Rbc);
  Eigen::Quaterniond qbc(Rbc);
  origin(0, 0) = qbc.w();
  origin(1, 0) = qbc.x();
  origin(2, 0) = qbc.y();
  origin(3, 0) = qbc.z();

  double image_width = _camera->GetImageWidth();
  double image_height = _camera->GetImageHeight();
  double height = _camera->GetHeight();
  Eigen::Vector3d image_center, camera_center;
  image_center << -image_width / 2, -image_height / 2, 0;
  _camera->ConvertImagePlanePoseToCamera(image_center, camera_center);

  origin(4, 0) = camera_center(0) * height;
  origin(5, 0) = camera_center(1) * height;
  origin(6, 0) = 0;
  return true;
}

double MapBuilder::GetMapResolution(){
  return _camera->GetLengthOfPixel();
}

OccupancyData& MapBuilder::GetMapData(){
  return _map_stitcher->GetOccupancyData();
}

OccupancyMap& MapBuilder::GetOccupancyMap(){
  return _map_stitcher->GetOccupancyMay();
}