#ifndef CAMERA_H_
#define CAMERA_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

class Camera{
public:
  Camera();
  Camera(const std::string& camera_file);
  Camera& operator=(const Camera& camera); // deep copy
  
  void UndistortImage(cv::Mat& image, cv::Mat& undistort_image);
  void GetNewCameraMatrix(cv::Mat& camera_matrix);
  double GetImageHeight();
  double GetImageWidth();
  double GetHeight();
  bool HeightIsAccurate();
  void GetExtrinsics(Eigen::Matrix3d& extrinsics);
  double GetLengthOfPixel();

  // image plane: pixel plane, image center is the origin 
  // Camera: normalized plane, 
  // Robot: robot body coordinate system.
  bool ConvertImagePlanePoseToCamera(Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& camera_pose);
  bool ConvertCameraPoseToImagePlane(Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& camera_pose);
  bool ConvertCameraPoseToRobot(Eigen::Vector3d& camera_pose, Eigen::Vector3d& robot_pose);
  bool ConvertRobotPoseToCamera(Eigen::Vector3d& camera_pose, Eigen::Vector3d& robot_pose);
  bool ConvertImagePlanePoseToRobot(Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& robot_pose);
  bool ConvertRobotPoseToImagePlane(Eigen::Vector3d& image_plane_pose, Eigen::Vector3d& robot_pose);

private:
  int _image_height;
  int _image_width;
  double _height;
  bool _accurate_height;
  cv::Mat _K;
  cv::Mat _new_K;
  cv::Mat _D;
  cv::Mat _map1;
  cv::Mat _map2;
  Eigen::Matrix3d _extrinsics;
};

typedef std::shared_ptr<Camera> CameraPtr;

#endif  // CAMERA_H_