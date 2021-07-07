#include "map_stitcher.h"
#include "utils.h"

#include <algorithm>

MapStitcher::MapStitcher(MapStitcherConfig config, CameraPtr camera): 
     _cell_size(config.cell_size), _camera(camera){
}

void MapStitcher::InsertFrame(FramePtr frame, cv::Mat& image){
  const double scale = 100.0 / 255.0;
  cv::Mat norm_image = image * scale;
  Eigen::MatrixXi matrix;
  cv::cv2eigen(norm_image, matrix);
  _raw_images[frame] = matrix;

  AddImageToOccupancy(frame);
}

Eigen::Vector2i MapStitcher::ComputeCellPosition(int x){
  Eigen::Vector2i result;
  if(x >=0 ){
    result(0) = x / _cell_size;
  }else{
    result(0) = (x - _cell_size + 1) / _cell_size;
  }
  result(1) = x - result(0) * _cell_size;

  return result;
}

void MapStitcher::AddImageToOccupancy(FramePtr frame){
  Eigen::MatrixXi& data = _raw_images[frame];
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
  double cx = (double)W / 2;
  double cy = (double)H / 2;
  for(int i = 0; i < W; i++){
    W_idx(i) = i - cx;
    X(i) = image_pose(0);
    Y(i) = image_pose(1);
  } 
  for(int i = 0; i < H; i++) H_idx(i) = i - cy;

  Eigen::VectorXd Wx = R(0, 0) * W_idx + X;
  Eigen::VectorXd Wy = R(1, 0) * W_idx + Y;
  Eigen::VectorXd Hx = R(0, 1) * H_idx;
  Eigen::VectorXd Hy = R(1, 1) * H_idx;

  // find which cells may be used
  int x00 = static_cast<int>((Wx(0) + Hx(0)));
  int y00 = static_cast<int>((Wy(0) + Hy(0)));
  int x01 = static_cast<int>((Wx(0) + Hx((H-1))));
  int y01 = static_cast<int>((Wy(0) + Hy((H-1))));
  int x10 = static_cast<int>((Wx((W-1)) + Hx(0)));
  int y10 = static_cast<int>((Wy((W-1)) + Hy(0)));
  int x11 = static_cast<int>((Wx((W-1)) + Hx((H-1))));
  int y11 = static_cast<int>((Wy((W-1)) + Hy((H-1))));
  
  int min_x = std::min(x00, std::min(x01, std::min(x10, x11)));
  int max_x = std::max(x00, std::max(x01, std::max(x10, x11)));
  int min_y = std::min(y00, std::min(y01, std::min(y10, y11)));
  int max_y = std::max(y00, std::max(y01, std::max(y10, y11)));

  Eigen::Vector2i min_cell_x = ComputeCellPosition(min_x);
  Eigen::Vector2i max_cell_x = ComputeCellPosition(max_x);
  Eigen::Vector2i min_cell_y = ComputeCellPosition(min_y);
  Eigen::Vector2i max_cell_y = ComputeCellPosition(max_y);

  std::unordered_map<GridLocation, bool, GridLocationHash, GridLocationEqual> locations;
  OccupancyData tmp_data;
  for(int i = min_cell_x(0); i <= max_cell_x(0); i++){
    for(int j = min_cell_y(0); j <= max_cell_y(0); j++){
      GridLocation grid_location(i, j);
      tmp_data[grid_location] = Cell(_cell_size);
      locations[grid_location] = false;
    }
  }

  for(int i = 0; i < W; i++){
    for(int j = 0; j < H; j ++){
      int x = static_cast<int>((Wx(i) + Hx(j)));
      int y = static_cast<int>((Wy(i) + Hy(j)));

      Eigen::Vector2i cell_x = ComputeCellPosition(x);
      Eigen::Vector2i cell_y = ComputeCellPosition(y);

      GridLocation grid_location(cell_x(0), cell_y(0));
      tmp_data[grid_location].data(cell_y(1), cell_x(1)) += data(j, i);
      tmp_data[grid_location].weight(cell_y(1), cell_x(1)) += 1;
      locations[grid_location] = true;
    }
  }

  for(auto& kv : tmp_data){
    GridLocation loc = kv.first;
    if(!locations[loc]) continue;
    if(_occupancy_data.count(loc) > 0){
      _occupancy_data[loc].size = _cell_size;
      _occupancy_data[loc].data = 
          _occupancy_data[loc].data * _occupancy_data[loc].weight + tmp_data[loc].data * tmp_data[loc].weight;
      _occupancy_data[loc].weight = _occupancy_data[loc].weight + tmp_data[loc].weight;
      for(int i = 0; i < _cell_size; i++){
        for(int j = 0; j < _cell_size; j++){
          if(_occupancy_data[loc].weight(i, j) < 1) continue;
          _occupancy_data[loc].data(i, j) = _occupancy_data[loc].data(i, j) / _occupancy_data[loc].weight(i, j);
        }
      }
    }else{
      _occupancy_data[loc].size = _cell_size;
      _occupancy_data[loc].data = tmp_data[loc].data;
      _occupancy_data[loc].weight = tmp_data[loc].weight;
    }
  }
}

void MapStitcher::RecomputeOccupancy(){
  _occupancy_data.clear();
  for(auto kv : _raw_images){
    AddImageToOccupancy(kv.first);
  }
}

OccupancyData& MapStitcher::GetOccupancyData(){
  return _occupancy_data;
}