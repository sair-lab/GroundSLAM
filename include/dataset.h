#ifndef DATASET_H_
#define DATASET_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "utils.h"

class Dataset{
public:
  Dataset(const std::string& dataroot);
  size_t GetDatasetLength();
  bool GetImage(cv::Mat& image, size_t idx);
  bool PoseIsAvailable();
  bool GetPose(Eigen::Vector3d& pose, size_t idx);
  double GetTimestamp(size_t idx);

private:
  std::string _dataroot;
  std::string _image_dir;
  std::string _pose_file_path;
  Aligned<std::vector, Eigen::Vector3d> _poses;
  std::string _image_name_file_path;
  std::vector<std::string> _image_names;
  std::string _time_file_path;
  std::vector<double> _timestamps;
};

#endif // DATASET_H_