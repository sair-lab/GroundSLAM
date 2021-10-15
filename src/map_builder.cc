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
    OdomPoseIsAvailable(odom_is_available), _init(false), _frame_id(0), _edge_id(0), 
    _kfs_config(configs.keyframe_selection_config){
  _camera = std::shared_ptr<Camera>(new Camera(configs.dataset_config.camera_file));
  _correlation_flow = std::shared_ptr<CorrelationFlow>(new CorrelationFlow(configs.cf_config));
  _map = std::shared_ptr<Map>(new Map(configs.map_config));

  _loop_closure = std::shared_ptr<LoopClosure>(
      new LoopClosure(configs.loop_closure_config, _correlation_flow, _map));

  _map_stitcher = std::shared_ptr<MapStitcher>(new MapStitcher(configs.map_stitcher_config, _camera));
}

bool MapBuilder::AddNewInput(cv::Mat& image, Eigen::Vector3d& odom_pose){

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
    return true;
  }

  Eigen::Vector3d response;
  bool good_tracking = Tracking(response);
  if(good_tracking || OdomPoseIsAvailable){
    // keyframe selection
    UpdateCurrentPose();
    SetCurrentFramePose();
    Eigen::Vector2d da = ComputeRelativeDA();
    bool c1 = da(0) > _kfs_config.max_distance;
    bool c2 = da(1) > _kfs_config.max_angle;
    bool c3 = ((response(0) > _kfs_config.lower_response_thr) && (response(0) < _kfs_config.upper_response_thr));
    bool c4 = ((response(2) > _kfs_config.lower_response_thr) && (response(2) < _kfs_config.upper_response_thr));
    bool to_insert = (c1 || c2 || c3 || c4);
    if(!to_insert) return false;
    _distance += da(0);
  }else{
    return false;
  }

  if(good_tracking) AddCFEdge();
  AddOdomEdgeToMap();
  _map->AddFrame(_current_frame);
  SetFrameDistance();
  _map_stitcher->InsertFrame(_current_frame, undistort_image);
  bool loop_found = FindLoopClosure();
  if(!loop_found){
    CheckAndOptimize();
  }

  UpdateIntermedium();

  return true;
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

void MapBuilder::CheckAndOptimize(){
  if(_loop_matches.size() >= 2){
    AddLoopEdges();
    OptimizeMap();
    _map_stitcher->RecomputeOccupancy();
  }
  _loop_matches.clear();
}

void MapBuilder::UpdateCurrentPose(){
  Eigen::Vector3d last_robot_pose_from_cf, current_robot_pose_from_cf;
  _camera->ConvertImagePlanePoseToRobot(_last_cf_pose, last_robot_pose_from_cf);
  _camera->ConvertImagePlanePoseToRobot(_current_cf_pose, current_robot_pose_from_cf);
  Eigen::Vector3d relativate_pose_from_cf = ComputeRelativePose(last_robot_pose_from_cf, current_robot_pose_from_cf);
  // Eigen::Vector3d relative_odom_pose = ComputeRelativePose(_last_odom_pose, _current_odom_pose);

  Eigen::Vector3d relative_pose = relativate_pose_from_cf;
  // relative_pose(2) = relative_odom_pose(2);
  _current_pose = ComputeAbsolutePose(_last_pose, relative_pose);
  // _camera->ConvertImagePlanePoseToRobot(_current_cf_pose, _current_pose);
}

bool MapBuilder::Tracking(Eigen::Vector3d& response){
  Eigen::Vector3d relative_pose;
  response = _correlation_flow->ComputePose(
      _last_fft_result, _image_array, _last_fft_polar, _fft_polar, relative_pose);

  bool good_tracking = ((response(0) > _kfs_config.lower_response_thr) && 
      ((response(2) > _kfs_config.lower_response_thr)));

  if(good_tracking){
    _current_cf_pose = ComputeAbsolutePose(_last_cf_pose, relative_pose);
    _camera->ConvertImagePlanePoseToCamera(_current_cf_pose, _current_cf_real_pose);
  }else if(OdomPoseIsAvailable){
    _camera->ConvertRobotPoseToCamera(_current_cf_real_pose, _current_odom_pose);
    _camera->ConvertCameraPoseToImagePlane(_current_cf_pose, _current_cf_real_pose);
  }

  return good_tracking;
}

void MapBuilder::AddCFEdge(){
  Eigen::Vector3d relative_cf_real_pose = ComputeRelativePose(_last_cf_real_pose, _current_cf_real_pose);
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
  AddCFEdgeToMap(relative_cf_real_pose, _last_frame->GetFrameId(), 
      _current_frame->GetFrameId(), _edge_id++, Edge::Type::KCC, info);
}

void MapBuilder::AddCFEdgeToMap(Eigen::Vector3d& relative_pose, int from, int to, 
    int edge_id, Edge::Type edge_type, Eigen::Matrix3d& info){
  EdgePtr cf_edge = std::make_shared<Edge>();
  cf_edge->_edge_id = edge_id;
  cf_edge->_type = edge_type;
  cf_edge->_from = from;
  cf_edge->_to = to;
  cf_edge->_T = relative_pose;
  cf_edge->_information = info;
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
  _map->AddEdge(odom_edge);
}

Eigen::Vector2d MapBuilder::ComputeRelativeDA(){
  Eigen::Vector3d rlt_camera_pose;
  if(OdomPoseIsAvailable){
    Eigen::Vector3d rlt_pose = _current_odom_pose - _last_odom_pose;
    _camera->ConvertRobotPoseToCamera(rlt_camera_pose, rlt_pose);
  }else{
    Eigen::Vector3d rlt_pose = _current_cf_pose - _last_cf_pose;
    _camera->ConvertImagePlanePoseToCamera(rlt_pose, rlt_camera_pose);
  }

  double d = sqrt(rlt_camera_pose.head(2).dot(rlt_camera_pose.head(2)));

  Eigen::Vector2d result;
  result << d, std::abs(rlt_camera_pose(2));
  return result;
}

void MapBuilder::SetFrameDistance(){
  _map->SetFrameDistance(_current_frame, _distance);
}

bool MapBuilder::FindLoopClosure(){
  LoopClosureResult loop_closure_result = 
      _loop_closure->FindLoopClosure(_image_array, _current_frame, _current_pose);
  if(loop_closure_result.found){
    _loop_matches.emplace_back(loop_closure_result);
    std::cout << "Find a loop edge, current frame = " << loop_closure_result.current_frame->GetFrameId()
              << ", loop frame = " << loop_closure_result.loop_frame->GetFrameId() << std::endl;
  }

  return loop_closure_result.found;
}

void MapBuilder::AddLoopEdges(){
  for(auto loop_match : _loop_matches){
    int edge_id = _edge_id++;
    int from = loop_match.loop_frame->GetFrameId();
    int to = loop_match.current_frame->GetFrameId();
    Eigen::Vector3d relative_pose;
    _camera->ConvertImagePlanePoseToCamera(loop_match.relative_pose, relative_pose);  
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    AddCFEdgeToMap(relative_pose, from, to, edge_id, Edge::Type::Loop, info);
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


  bool has_odom_edge = false;
  std::vector<int> scale_data_idx;
  for(EdgePtr& edge : edges){
    int from = edge->_from;
    int to = edge->_to;
    if((frame_ids.count(from) < 1) || (frame_ids.count(to) < 1)){
      continue;
    }

    Eigen::Vector3d relative_pose;
    if(edge->_type == Edge::Type::KCC || edge->_type == Edge::Type::Loop){
      scale_data_idx.emplace_back(0);
      _camera->ConvertCameraPoseToRobot(edge->_T, relative_pose);
    }else if(edge->_type == Edge::Type::Odom && OdomPoseIsAvailable){
      scale_data_idx.emplace_back(1);
      relative_pose = edge->_T;
      has_odom_edge = true;
    }else{
      continue;
    }

    if(has_odom_edge){
      ceres::optimization_2d::ScaleData odom_scale(1.0, true);
      scale_data.emplace_back(odom_scale);
    }

    ceres::optimization_2d::Constraint2d constraint;
    constraint.id_begin = from;
    constraint.id_end = to;
    constraint.x = relative_pose(0);
    constraint.y = relative_pose(1);
    constraint.yaw_radians = relative_pose(2);
    constraint.information = edge->_information;
    constraints.emplace_back(constraint);
  }

  ceres::Problem problem;
  ceres::optimization_2d::BuildOptimizationProblem(constraints, &poses, &problem);
  // ceres::optimization_2d::BuildOptimizationProblemWithScale(constraints, scale_data, scale_data_idx, &poses, &problem);
  CHECK(ceres::optimization_2d::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  // copy back frame pose
  AlignedMap<int, Eigen::Vector3d> frame_poses;
  for(auto kv : poses){
    Eigen::Vector3d frame_pose;
    frame_pose << kv.second.x, kv.second.y, kv.second.yaw_radians;
    frame_poses[kv.first] = frame_pose;
  }
  _map->UpdatePoses(frame_poses);

  std::cout << "Map optimization is done !" << std::endl;

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

bool MapBuilder::GetOccupancyMapOrigin(
    Eigen::Vector3d& pixel_origin, Eigen::Matrix<double, 7, 1>& real_origin){
  Eigen::Matrix3d Rbc;
  _camera->GetExtrinsics(Rbc);
  Eigen::Quaterniond qbc(Rbc);
  real_origin(0, 0) = qbc.w();
  real_origin(1, 0) = qbc.x();
  real_origin(2, 0) = qbc.y();
  real_origin(3, 0) = qbc.z();

  Eigen::Vector3d robot_origin;
  _camera->ConvertImagePlanePoseToRobot(pixel_origin, robot_origin);

  real_origin(4, 0) = robot_origin(0);
  real_origin(5, 0) = robot_origin(1);
  real_origin(6, 0) = robot_origin(2);

  return true;
}

double MapBuilder::GetMapResolution(){
  return _camera->GetLengthOfPixel();
}

OccupancyData& MapBuilder::GetMapData(){
  return _map_stitcher->GetOccupancyData();
}