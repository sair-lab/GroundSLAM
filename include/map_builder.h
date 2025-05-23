#ifndef MAP_BUILDER_H_
#define MAP_BUILDER_H_

#include "read_configs.h"
#include "camera.h"
#include "frame.h"
#include "correlation_flow.h"
#include "map.h"
#include "loop_closure.h"
#include "map_stitcher.h"


class MapBuilder{
public:
  MapBuilder(Configs& configs);

  bool AddNewInput(cv::Mat& image, double timestamp = -1);
  void ComputeFFTResult(cv::Mat& image);
  void ConstructFrame(double timestamp);
  void SetCurrentFramePose();
  bool Initialize();
  void UpdateIntermedium();
  void UpdateCurrentPose();
  bool Tracking(Eigen::Vector3d& response);
  void AddCFEdge();
  void AddCFEdgeToMap(Eigen::Vector3d& relative_pose, int from, int to, 
      int edge_id, Edge::Type edge_type, Eigen::Matrix3d& info);
  Eigen::Vector2d ComputeRelativeDA();
  void SetFrameDistance();
  bool FindLoopClosure();
  void AddLoopEdges();
  bool OptimizeMap();
  void CheckAndOptimize();
  void UpdateValueAfterLoop();

  // for visualization
  bool GetCFPose(Eigen::Vector3d& pose); 
  bool GetFramePoses(Aligned<std::vector, Eigen::Vector3d>& poses, std::vector<double>& timestamps);
  bool GetOccupancyMapOrigin(
      Eigen::Vector3d& pixel_origin, Eigen::Matrix<double, 7, 1>& real_origin);  // [qw, qx, qy, qz, x, y, z]
  double GetMapResolution();
  OccupancyData& GetMapData();

private:
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
  // robot pose
  Eigen::Vector3d _last_pose;
  Eigen::Vector3d _current_pose;
  // distance
  double _distance;

  // intermedium rsults
  Eigen::ArrayXXf _image_array;
  Eigen::ArrayXXcf _last_fft_result;
  Eigen::ArrayXXcf _last_fft_polar;
  Eigen::ArrayXXcf _fft_result;
  Eigen::ArrayXXcf _fft_polar;
  std::vector<LoopClosureResult> _loop_matches;

  Configs _configs;
  CameraPtr _camera;
  CorrelationFlowPtr _correlation_flow;
  KeyframeSelectionConfig _kfs_config;
  MapPtr _map;
  LoopClosurePtr _loop_closure;
  MapStitcherPtr _map_stitcher;
};


#endif  // MAP_BUILDER_H
