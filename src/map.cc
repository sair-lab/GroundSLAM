#include <iostream>
#include <unistd.h>
#include <map>
#include <vector>

#include "map.h"


using namespace std;

Map::Map(): _grid_scale(1.0){
}

Map::Map(MapConfig& map_config): _grid_scale(map_config.grid_scale){
}

void Map::AddFrame(FramePtr& frame){
  if(_frames.size() < 1){
    _baseframe = frame;
    frame->SetFrameId(0);
  }

  int frame_id = frame->GetFrameId();
  _frames[frame_id] = frame;

  Eigen::Vector3d pose;
  frame->GetPose(pose);
  GridLocation grid_location = ComputeGridLocation(pose);
  _grid_map[grid_location].insert(frame);
}

void Map::SetFrameDistance(FramePtr& frame, double distance){
  _frame_distanses[frame] = distance;
}

void Map::AddEdge(EdgePtr& edge){
  int edge_id = edge->_edge_id;
  int from = edge->_from; 
  int to = edge->_to; 

  _edges[edge_id] = edge;
  if(_frames.count(from) > 0){
    _frames[from]->AddEdge(edge_id);
  }  

  if(_frames.count(to) > 0){
    _frames[to]->AddEdge(edge_id);
  }
}

int Map::GetAllFrames(std::vector<FramePtr>& frames){
  for(auto kv : _frames){
    frames.emplace_back(kv.second);
  }
  return frames.size();
}

double Map::GetFrameDistance(FramePtr& frame){
  if(_frame_distanses.count(frame) > 0){
    return _frame_distanses[frame];
  }else{
    return -1;
  }
}

int Map::GetAllEdges(std::vector<EdgePtr>& edges){
  for(auto kv : _edges){
    edges.emplace_back(kv.second);
  }
  return edges.size();
}

void Map::UpdatePoses(AlignedMap<int, Eigen::Vector3d> frame_poses){
  for(auto kv : frame_poses){
    if(_frames.count(kv.first) > 0){
      _frames[kv.first]->SetPose(kv.second);
    }
  }
}

GridLocation Map::ComputeGridLocation(double x, double y){
  int grid_x = static_cast<int>((x / _grid_scale));
  int grid_y = static_cast<int>((y / _grid_scale));
  return GridLocation(grid_x, grid_y);
}

GridLocation Map::ComputeGridLocation(Eigen::Vector3d pose){
  double x = pose(0);
  double y = pose(1);
  return ComputeGridLocation(x, y);
}

int Map::GetFramesInGrids(
    std::vector<FramePtr>& frames, std::vector<GridLocation>& grid_locations){
  for(auto grid_location : grid_locations){
    if(_grid_map.count(grid_location) > 0){
      frames.insert(frames.end(), _grid_map[grid_location].begin(), _grid_map[grid_location].end());
    }
  }
  return frames.size();
}

FramePtr Map::GetBaseframe(){
  return _baseframe;
}