#include "frame.h"

Frame::Frame(){
}

Frame::Frame(int frame_id): _frame_id(frame_id){
}

Frame::Frame(int frame_id, double timestamp, Eigen::ArrayXXf& frame, Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& fft_polar):
    _frame_id(frame_id), _timestamp(timestamp), _frame(frame), _fft_result(fft_result), _fft_polar(fft_polar){
}

Frame& Frame::operator=(const Frame& other){
  _frame_id = other._frame_id;
  _timestamp = other._timestamp;
  _fft_result = other._fft_result;
  _fft_polar = other._fft_polar;
  _pose = other._pose;
  _edge_ids = other._edge_ids;
  return *this;
}

void Frame::SetFrameId(int frame_id){
  _frame_id = frame_id;
}

int Frame::GetFrameId(){
  return _frame_id;
}

double Frame::GetTimestamp(){
  return _timestamp;
}

Eigen::ArrayXXf Frame::GetFrame(){
  return _frame;
}

void Frame::SetFFTResult(Eigen::ArrayXXcf& fft_result){
  _fft_result = fft_result;
}

void Frame::SetFFTResult(Eigen::ArrayXXcf& fft_result,
    Eigen::ArrayXXcf& fft_polar){
  _fft_result = fft_result;
  _fft_polar = fft_polar;
}

void Frame::GetFFTResult(Eigen::ArrayXXcf& fft_result){
  fft_result = _fft_result;
}

void Frame::GetFFTResult(Eigen::ArrayXXcf& fft_result,
    Eigen::ArrayXXcf& fft_polar){
  fft_result = _fft_result;
  fft_polar = _fft_polar;
}

void Frame::SetPose(Eigen::Vector3d& pose){
  _pose = pose;
}
void Frame::GetPose(Eigen::Vector3d& pose){
  pose = _pose;
}

void Frame::AddEdge(int edge_id){
  _edge_ids.emplace_back(edge_id);
}

void Frame::GetEdgeIds(std::vector<int>& edge_ids){
  edge_ids = _edge_ids;
}

void Frame::SaveToDisk(const std::string root_dir){
  //TODO
}