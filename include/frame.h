#ifndef FRAME_H_
#define FRAME_H_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "utils.h"


class Frame{
public:
  Frame();
  Frame(int frame_id);
  Frame(int frame_id, double timestamp, Eigen::ArrayXXf&, Eigen::ArrayXXcf&, Eigen::ArrayXXcf&);
  Frame& operator=(const Frame& other);

  void SetFrameId(int frame_id);
  int GetFrameId();
  double GetTimestamp();
  Eigen::ArrayXXf GetFrame();
  void SetFFTResult(Eigen::ArrayXXcf& fft_result);
  void SetFFTResult(Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& depth_fft_result);
  void GetFFTResult(Eigen::ArrayXXcf& fft_result);
  void GetFFTResult(Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& depth_fft_result);
  void SetPose(Eigen::Vector3d& pose);
  void GetPose(Eigen::Vector3d& pose);
  void AddEdge(int edge_id);
  void GetEdgeIds(std::vector<int>& edge_ids);
  void SaveToDisk(const std::string root_dir);

private:
  int _frame_id;
  double _timestamp;
  Eigen::ArrayXXf _frame;
  Eigen::ArrayXXcf _fft_result;
  Eigen::ArrayXXcf _fft_polar;
  Eigen::ArrayXXcf _depth_fft_result;
  Eigen::Vector3d _pose;
  std::vector<int> _edge_ids;
};

typedef std::shared_ptr<Frame> FramePtr;

#endif  // FRAME_H_