#include <yaml-cpp/yaml.h>

#include "camera.h"
#include "utils.h"

Camera::Camera(){
}

Camera::Camera(const std::string& camera_file){
  if(!FileExists(camera_file)){
    std::cout << "camera file: " << camera_file << " doesn't exist" << std::endl;
    return;
  }
  YAML::Node file_node = YAML::LoadFile(camera_file);

  _image_width = file_node["image_size"][0].as<int>();
  _image_height = file_node["image_size"][1].as<int>();
  _height = file_node["height"].as<double>();
  _accurate_height = file_node["accurate_height"].as<bool>();

  YAML::Node K_node = file_node["intrinsics"]["data"];
  double fx = K_node[0].as<double>();
  double cx = K_node[1].as<double>();
  double fy = K_node[2].as<double>();
  double cy = K_node[3].as<double>();
  _K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

  YAML::Node D_node = file_node["distortion"]["data"];
  double D_data[5];
  for(size_t i = 0; i < 5; i++){
    D_data[i] = D_node[i].as<double>();
  }
  _D = (cv::Mat_<double>(5, 1) << D_data[0], D_data[1], D_data[2], D_data[3], D_data[4]);

  cv::Size image_size(_image_width, _image_height);
  _new_K = getOptimalNewCameraMatrix(_K, _D, image_size, 0, image_size);
  initUndistortRectifyMap(_K, _D, cv::Mat(), _new_K, image_size, CV_16SC2, _map1, _map2);

  YAML::Node E_node = file_node["extrinsics"]["data"];
  for(size_t i = 0; i < 3; i++){
    for(size_t j = 0; j < 3; j++){
      size_t idx = 3 * i + j;
      _extrinsics(i, j) = E_node[idx].as<double>();
    }
  }
}

Camera& Camera::operator=(const Camera& camera){
  _image_height = camera._image_height;
  _image_width = camera._image_width;
  _height = camera._height;
  _accurate_height = camera._accurate_height;
  _K = camera._K.clone();
  _new_K = camera._new_K;
  _D = camera._D.clone();
  _map1 = camera._map1.clone();
  _map2 = camera._map2.clone();
  _extrinsics = camera._extrinsics;
  return *this;
}

void Camera::UndistortImage(cv::Mat& image, cv::Mat& undistort_image){
  remap(image, undistort_image, _map1, _map2, cv::INTER_LINEAR);
}

void Camera::GetNewCameraMatrix(cv::Mat& camera_matrix){
  camera_matrix = _new_K.clone();
}

double Camera::GetImageHeight(){
  return _image_height;
}

bool Camera::HeightIsAccurate(){
  return _accurate_height;
}

double Camera::GetImageWidth(){
  return _image_width;
}

double Camera::GetHeight(){
  return _height;
}

void Camera::GetExtrinsics(Eigen::Matrix3d& extrinsics){
  extrinsics = _extrinsics;
}

double Camera::GetLengthOfPixel(){
  Eigen::Vector3d pixel(1.0, 1.0, 0.0);
  Eigen::Vector3d real;
  ConvertImagePlanePoseToRobot(pixel, real);
  return (real(0) + real(1)) / 2;
}

bool Camera::ConvertImagePlanePoseToCamera(
    Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& camera_pose){
  double u = image_plane_pose(0);
  double v = image_plane_pose(1);
  double angle = image_plane_pose(2);

  double fx = _new_K.at<double>(0, 0);
  // double cx = _new_K.at<double>(0, 2);
  double fy = _new_K.at<double>(1, 1);
  // double cy = _new_K.at<double>(1, 2);

  double x = u / fx;
  double y = v / fy;

  camera_pose << x, y, angle;

  return true;
}

bool Camera::ConvertCameraPoseToImagePlane(
    Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& camera_pose){
  double x = camera_pose(0);
  double y = camera_pose(1);
  double angle = camera_pose(2);

  double fx = _new_K.at<double>(0, 0);
  // double cx = _new_K.at<double>(0, 2);
  double fy = _new_K.at<double>(1, 1);
  // double cy = _new_K.at<double>(1, 2); 

  double u = fx * x;
  double v = fy * y;

  image_plane_pose << u, v, angle;

  return true;
}

bool Camera::ConvertCameraPoseToRobot(
    Eigen::Vector3d& camera_pose, Eigen::Vector3d& robot_pose){
  if(_height < 0){
    std::cout << "camera height: " << _height << " < 0" << std::endl;
    return false;
  }

  double x = _height * camera_pose(0);
  double y = _height * camera_pose(1);

  robot_pose << x, y, camera_pose(2);
  robot_pose = _extrinsics * robot_pose;

  return true;
}

bool Camera::ConvertRobotPoseToCamera(
    Eigen::Vector3d& camera_pose, Eigen::Vector3d& robot_pose){
  if(_height < 0){
    std::cout << "camera height: " << _height << " < 0" << std::endl;
    return false;
  }

  camera_pose = _extrinsics.inverse() * robot_pose;
  camera_pose(0) /= _height;
  camera_pose(1) /= _height;
  return true;
}

bool Camera::ConvertImagePlanePoseToRobot(
    Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& robot_pose){
  Eigen::Vector3d camera_pose;
  bool c1 = ConvertImagePlanePoseToCamera(image_plane_pose, camera_pose);
  bool c2 = ConvertCameraPoseToRobot(camera_pose, robot_pose);

  return (c1 && c2);
}

bool Camera::ConvertRobotPoseToImagePlane(
    Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& robot_pose){
  Eigen::Vector3d camera_pose;
  bool c1 = ConvertRobotPoseToCamera(camera_pose, robot_pose);
  bool c2 = ConvertCameraPoseToImagePlane(image_plane_pose, camera_pose);

  return (c1 && c2);
}