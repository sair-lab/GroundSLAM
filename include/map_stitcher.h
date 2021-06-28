#ifndef MAP_STITCHER_H_
#define MAP_STITCHER_H_

#include "optimization_2d/pose_graph_2d_error_term.h"
#include "camera.h"
#include "frame.h"
#include "map.h"

typedef std::unordered_map<GridLocation, std::vector<int>, GridLocationHash, GridLocationEqual> OccupancyData;
typedef std::unordered_map<GridLocation, int, GridLocationHash, GridLocationEqual> OccupancyMap;
typedef std::unordered_set<GridLocation, GridLocationHash, GridLocationEqual> LocationSet;

class MapStitcher{
public:
  MapStitcher(CameraPtr camera);
  void InsertFrame(FramePtr frame, cv::Mat& image);
  void AddImageToOccupancy(FramePtr frame, LocationSet& locations);
  void RecomputeOccupancy();
  void UpdateMap(LocationSet& locations);
  OccupancyData& GetOccupancyData();
  OccupancyMap& GetOccupancyMay();


private:
  CameraPtr _camera;
  std::unordered_map<FramePtr, Eigen::MatrixXi> _raw_images;
  OccupancyData _occupancy_data;
  OccupancyMap _occupancy_map;
};

typedef std::shared_ptr<MapStitcher> MapStitcherPtr;

#endif  // MAP_STITCHER_H_