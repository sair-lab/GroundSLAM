#ifndef LOOP_CLOSURE_H_
#define LOOP_CLOSURE_H_

#include "read_configs.h"
#include "correlation_flow.h"
#include "map.h"

struct LoopClosureResult{
  bool found;
  Eigen::Vector3d response;
  FramePtr current_frame;
  FramePtr loop_frame;
  Eigen::Vector3d relative_pose;

  LoopClosureResult(): found(false), response(-1.0, -1.0, -1.0) {}
  LoopClosureResult(bool _found, Eigen::Vector3d _resopnse): found(_found), response(_resopnse) {}
  LoopClosureResult& operator=(const LoopClosureResult& other){
    found = other.found;
    response = other.response;
    current_frame = other.current_frame;
    loop_frame = other.loop_frame;
    relative_pose = other.relative_pose;
    return *this;
  }
};

class LoopClosure{
public:
  LoopClosure(LoopClosureConfig& loop_closure_config, CorrelationFlowPtr correlation_flow, MapPtr map);
  LoopClosureResult FindLoopClosure(Eigen::ArrayXXf& image, FramePtr& current_frame);
  LoopClosureResult FindLoopClosure(Eigen::ArrayXXf& image, FramePtr& current_frame, Eigen::Vector3d& prior_pose);
  LoopClosureResult FindLoopClosure(Eigen::ArrayXXf& image, FramePtr& current_frame, std::vector<FramePtr>& frames);

private:
  LoopClosureConfig _loop_thr;
  CorrelationFlowPtr _correlation_flow;
  MapPtr _map;
};

typedef std::shared_ptr<LoopClosure> LoopClosurePtr;

#endif  // LOOP_CLOSURE_H_