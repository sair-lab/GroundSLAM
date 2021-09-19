#include "dataset.h"
#include "utils.h"

Dataset::Dataset(const std::string& dataroot): _dataroot(dataroot){
  if(!PathExists(dataroot)){
    std::cout << "dataroot : " << dataroot << " doesn't exist" << std::endl;
    exit(0);
  }

  _image_dir = ConcatenateFolderAndFileName(_dataroot, "image_0");
  _pose_file_path = ConcatenateFolderAndFileName(_dataroot, "odom.txt");
  _image_name_file_path = ConcatenateFolderAndFileName(_dataroot, "image_names.txt");
  _time_file_path = ConcatenateFolderAndFileName(_dataroot, "times.txt");

  if(FileExists(_pose_file_path)){
    std::vector<std::vector<std::string> > poses_data;
    ReadTxt(_pose_file_path, poses_data, ",");
    for(std::vector<std::string>& line : poses_data){
      double x, y, angle;
      x = atof(line[0].c_str());
      y = atof(line[1].c_str());
      angle = atof(line[2].c_str());

      Eigen::Vector3d pose;
      pose << x, y, angle;
      _poses.emplace_back(pose);
    }
  }

  std::vector<std::vector<std::string> > image_names_data;
  ReadTxt(_image_name_file_path, image_names_data, ",");
  for(std::vector<std::string>& line : image_names_data){
    _image_names.emplace_back(line[0]);
  }

  std::vector<std::vector<std::string> > timestamps_data;
  if(FileExists(_time_file_path)){
    ReadTxt(_time_file_path, timestamps_data, ",");
    for(std::vector<std::string>& line : timestamps_data){
      _timestamps.push_back(atof(line[0].c_str()));
    }
  }
}

size_t Dataset::GetDatasetLength(){
  return _image_names.size();
}

bool Dataset::GetImage(cv::Mat& image, size_t idx){
  if(idx >= _image_names.size()){
    return false;
  }

  std::string image_name = _image_names[idx];
  std::string image_path = ConcatenateFolderAndFileName(_image_dir, image_name);
  image = cv::imread(image_path, cv::ImreadModes::IMREAD_GRAYSCALE);
  return true;
}

bool Dataset::PoseIsAvailable(){
  return (_poses.size() > 0);
}

bool Dataset::GetPose(Eigen::Vector3d& pose, size_t idx){
  if(idx >= _poses.size()){
    return false;
  }

  pose = _poses[idx];
  return true;
}

double Dataset::GetTimestamp(size_t idx){
  if(_timestamps.size() < idx){
    return _timestamps[idx];
  }else{
    return -1.0;
  }
}
