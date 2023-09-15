#include "loop_closure.h"

#include <cmath>

LoopClosure::LoopClosure(LoopClosureConfig& loop_closure_config, 
    CorrelationFlowPtr correlation_flow, MapPtr map): 
    _loop_thr(loop_closure_config),  _correlation_flow(correlation_flow), _map(map){
}

LoopClosureResult LoopClosure::FindLoopClosure(Eigen::ArrayXXf& image, FramePtr& current_frame){
  std::vector<FramePtr> frames;
  _map->GetAllFrames(frames);
  LoopClosureResult result = FindLoopClosure(image, current_frame, frames);
  return result;
}

LoopClosureResult LoopClosure::FindLoopClosure(
    Eigen::ArrayXXf& image, FramePtr& current_frame, Eigen::Vector3d& prior_pose){
  std::vector<GridLocation> grid_locations;
  GridLocation grid_location = _map->ComputeGridLocation(prior_pose);
  for(int i = -1; i <= 1; i++){
    for(int j = -1; j <= 1; j++){
      GridLocation gl = grid_location;
      gl.x += i;
      gl.y += j;
      grid_locations.emplace_back(gl);
    }
  }

  std::vector<FramePtr> frames; 
  _map->GetFramesInGrids(frames, grid_locations);
  LoopClosureResult result = FindLoopClosure(image, current_frame, frames);
  return result;
}

LoopClosureResult LoopClosure::FindLoopClosure(
    Eigen::ArrayXXf& image, FramePtr& current_frame, std::vector<FramePtr>& frames){
  Eigen::ArrayXXcf current_fft_result, current_fft_polar;
  current_frame->GetFFTResult(current_fft_result, current_fft_polar);
  LoopClosureResult result;
  result.current_frame = current_frame;
  for(FramePtr frame : frames){
    if(_loop_thr.frame_gap_thr > 0 && 
        std::abs((current_frame->GetFrameId() - frame->GetFrameId())) < _loop_thr.frame_gap_thr){
      continue;
    }
    if(_loop_thr.distance_thr > 0){
      double d1 = _map->GetFrameDistance(current_frame);   // distance is accumulation distance
      double d2 = _map->GetFrameDistance(frame);
      if(std::abs((d1 - d2)) < _loop_thr.distance_thr){
        continue;
      }
    }

    Eigen::ArrayXXcf fft_result, fft_polar;
    frame->GetFFTResult(fft_result, fft_polar);
    Eigen::Vector3d relative_pose;
    Eigen::Vector3d response = 
        _correlation_flow->ComputePose(fft_result, image, fft_polar, current_fft_polar, relative_pose, false);

    if(response.sum() > result.response.sum()){
      result.response = response;
      result.loop_frame = frame;
      result.relative_pose = relative_pose;
    }
  }

  bool c1 = (result.response(0) > _loop_thr.position_response_thr);
  bool c2 = (result.response(2) > _loop_thr.angle_response_thr);

  result.found = (c1 && c2);
  return result;
}