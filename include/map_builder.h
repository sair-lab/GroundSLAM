#ifndef MAP_BUILDER_H_
#define MAP_BUILDER_H

#include "read_configs.h"
#include "camera.h"
#include "frame.h"
#include "correlation_flow.h"
#include "map.h"
#include "loop_closure.h"
#include "map_stitcher.h"

class MapBuilder{
public:
  MapBuilder(Configs& configs, const bool odom_is_available);

  bool AddNewInput(cv::Mat& image, Eigen::Vector3d& odom_pose);
  void ComputeFFTResult(cv::Mat& image);
  void ConstructFrame();
  void SetCurrentFramePose();
  bool Initialize();
  void UpdateIntermedium();
  void UpdateCurrentPose();
  bool Tracking();
  void AddCFEdge();
  void AddCFEdgeToMap(Eigen::Vector3d& relative_pose, 
      int from, int to, int edge_id, Eigen::Matrix3d& info);
  void AddOdomEdgeToMap();
  Eigen::Vector2d ComputeRelativeDA();
  void SetFrameDistance();
  bool FindLoopClosure();
  void AddLoopEdges();
  bool OptimizeMap();
  void CheckAndOptimize();

  // for visualization
  bool GetOdomPose(Eigen::Vector3d& pose);  // odom pose in baseframe
  bool GetCFPose(Eigen::Vector3d& pose);    // rlt robot pose 
  bool GetFramePoses(Aligned<std::vector, Eigen::Vector3d>& poses);
  bool GetOccupancyMapOrigin(
      Eigen::Vector3d& pixel_origin, Eigen::Matrix<double, 7, 1>& real_origin);  // [qw, qx, qy, qz, x, y, z]
  double GetMapResolution();
  OccupancyData& GetMapData();

private:
  const bool OdomPoseIsAvailable;
  bool _init;
  int _frame_id;
  int _edge_id;
  bool _last_lost;
  
  // tmp
  FramePtr _last_frame;
  FramePtr _current_frame;
  // image plane pose
  Eigen::Vector3d _last_cf_pose;
  Eigen::Vector3d _current_cf_pose;
  // real scale pose
  Eigen::Vector3d _last_cf_real_pose;
  Eigen::Vector3d _current_cf_real_pose;
  // odom pose
  Eigen::Vector3d _baseframe_odom_pose;
  Eigen::Vector3d _last_odom_pose;
  Eigen::Vector3d _current_odom_pose;
  // robot pose
  Eigen::Vector3d _last_pose;
  Eigen::Vector3d _current_pose;
  // distance
  double _distance;

  // Intermedium rsults
  Eigen::ArrayXXf _image_array;
  Eigen::ArrayXXcf _last_fft_result;
  Eigen::ArrayXXcf _last_fft_polar;
  Eigen::ArrayXXcf _fft_result;
  Eigen::ArrayXXcf _fft_polar;
  std::vector<LoopClosureResult> _loop_matches;

  CameraPtr _camera;
  CorrelationFlowPtr _correlation_flow;
  KeyframeSelectionConfig _kfs_config;
  MapPtr _map;
  LoopClosurePtr _loop_closure;
  MapStitcherPtr _map_stitcher;
};


#endif  // MAP_BUILDER_H
