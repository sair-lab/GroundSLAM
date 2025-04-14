#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include "read_configs.h"
#include "camera.h"
#include "frame.h"
#include "edge.h"
#include "utils.h"


struct GridLocation{
  GridLocation() : x(0), y(0) {}
  GridLocation(int x_, int y_) : x(x_), y(y_) {}
  GridLocation& operator=(const GridLocation& other){
    x = other.x;
    y = other.y;
    return *this;
  }
  friend std::ostream & operator<<(std::ostream &out, GridLocation &loc){
    out << "(" << loc.x << ", " << loc.y << ")";
    return out;
  }

  int x;
  int y;
};

struct GridLocationHash{
  std::size_t operator() (const GridLocation& loc) const{
    return std::hash<int>()(loc.x) ^ std::hash<int>()(loc.y);
  }
};

struct GridLocationEqual{
  bool operator()(const GridLocation& l1, const GridLocation& l2) const{
    return l1.x == l2.x && l1.y == l2.y;
  }
};

typedef std::unordered_set<FramePtr> FrameSet;
typedef std::unordered_map<GridLocation, FrameSet, GridLocationHash, GridLocationEqual> GridMap;

class Map{
public:
  Map();
  Map(MapConfig& map_config);

  void AddFrame(FramePtr& frame);
  void SetFrameDistance(FramePtr& frame, double distance);
  void AddEdge(EdgePtr& edge);
  
  int GetAllFrames(std::vector<FramePtr>& frames);
  double GetFrameDistance(FramePtr& frame);
  int GetAllEdges(std::vector<EdgePtr>& edges);

  void UpdatePoses(AlignedMap<int, Eigen::Vector3d> frame_poses);

  GridLocation ComputeGridLocation(double x, double y);
  GridLocation ComputeGridLocation(Eigen::Vector3d pose);
  int GetFramesInGrids(std::vector<FramePtr>& frames, std::vector<GridLocation>& grid_locations);

  FramePtr GetBaseframe();

private:
  CameraPtr _camera;
  std::map<int, FramePtr> _frames;
  std::map<FramePtr, double> _frame_distanses;
  std::map<int, EdgePtr> _edges;
  double _grid_scale;
  GridMap _grid_map;
  FramePtr _baseframe;
};

typedef std::shared_ptr<Map> MapPtr;

#endif  // MAP_H_