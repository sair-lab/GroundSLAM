#include "map_stitcher.h"
#include "utils.h"

MapStitcher::MapStitcher(CameraPtr camera): _camera(camera){
}

void MapStitcher::InsertFrame(FramePtr frame, cv::Mat& image){
  const double scale = 100.0 / 255.0;
  cv::Mat norm_image = image * scale;
  Eigen::MatrixXi matrix;
  cv::cv2eigen(norm_image, matrix);
  _raw_images[frame] = matrix;

  LocationSet update_locations;
  AddImageToOccupancy(frame, update_locations);
  // UpdateMap(update_locations);
}

void MapStitcher::AddImageToOccupancy(FramePtr frame, LocationSet& locations){
  Eigen::MatrixXi& data = _raw_images[frame];
  
  // TODO: need pose of image plane, to change
  Eigen::Vector3d robot_pose, image_pose;
  frame->GetPose(robot_pose);
  _camera->ConvertRobotPoseToImagePlane(image_pose, robot_pose);

  int W = data.cols();
  int H = data.rows();
  Eigen::Matrix2d R = ceres::optimization_2d::RotationMatrix2D(image_pose(2));

  Eigen::VectorXd W_idx(W);
  Eigen::VectorXd H_idx(H);
  Eigen::VectorXd X(W);
  Eigen::VectorXd Y(W);
  for(int i = 0; i < W; i++){
    W_idx(i) = i;
    X(i) = image_pose(0);
    Y(i) = image_pose(1);
  } 
  for(int i = 0; i < H; i++) H_idx(i) = i;

  Eigen::VectorXd Wx = R(0, 0) * W_idx + X;
  Eigen::VectorXd Wy = R(1, 0) * W_idx + Y;
  Eigen::VectorXd Hx = R(0, 1) * H_idx;
  Eigen::VectorXd Hy = R(1, 1) * H_idx;

  for(int i = 0; i < W; i++){
    for(int j = 0; j < H; j ++){
      int x = static_cast<int>((Wx(i) + Hx(j)));
      int y = static_cast<int>((Wy(i) + Hy(j)));
      GridLocation grid_location(x, y);
      _occupancy_data[grid_location].emplace_back(data(j, i));
      // locations.insert(grid_location);
    }
  }

  std::cout << "_occupancy_data size = " << _occupancy_data.size() << std::endl;
}

void MapStitcher::RecomputeOccupancy(){
  _occupancy_data.clear();
  _occupancy_map.clear();
  LocationSet locations;
  for(auto kv : _raw_images){
    AddImageToOccupancy(kv.first, locations);
  }
  // UpdateMap(locations);
}

OccupancyData& MapStitcher::GetOccupancyData(){
  return _occupancy_data;
}

void MapStitcher::UpdateMap(LocationSet& locations){
  for(auto location : locations){
    if(_occupancy_data.count(location) < 1) continue;
    _occupancy_map[location] = 
        std::accumulate(_occupancy_data[location].begin(), _occupancy_data[location].end(), 0) / _occupancy_data[location].size();
  }
}

OccupancyMap& MapStitcher::GetOccupancyMay(){
  return _occupancy_map;
}