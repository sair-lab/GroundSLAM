#ifndef MAP_STITCHER_H_
#define MAP_STITCHER_H_

#include "optimization_2d/pose_graph_2d_error_term.h"
#include "camera.h"
#include "frame.h"
#include "map.h"
#include "read_configs.h"

struct Cell{
  int size;
  Eigen::ArrayXXi data;
  Eigen::ArrayXXi weight;

  Cell(): size(0) {}
  Cell(int _size): size(_size), data(Eigen::ArrayXXi::Zero(_size, _size)), weight(Eigen::ArrayXXi::Zero(_size, _size)) {}
};

typedef std::unordered_map<GridLocation, Cell, GridLocationHash, GridLocationEqual> OccupancyData;

class MapStitcher{
public:
  MapStitcher(MapStitcherConfig config, CameraPtr camera);
  void InsertFrame(FramePtr frame, cv::Mat& image);
  // input is the posion in one axis, output is the cell and the position in cell
  Eigen::Vector2i ComputeCellPosition(int x);
  void AddImageToOccupancy(FramePtr frame);
  void RecomputeOccupancy();
  OccupancyData& GetOccupancyData();

private:
  int _cell_size;
  bool _to_stitch;
  CameraPtr _camera;
  std::unordered_map<FramePtr, Eigen::MatrixXi> _raw_images;
  OccupancyData _occupancy_data;
};

typedef std::shared_ptr<MapStitcher> MapStitcherPtr;

#endif  // MAP_STITCHER_H_