
#include "utils.h"
#include "optimization_2d/pose_graph_2d_error_term.h"

bool FileExists(const std::string& file) {
  struct stat file_status;
  if (stat(file.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFREG)) {
    return true;
  }
  return false;
}


bool PathExists(const std::string& path) {
  struct stat file_status;
  if (stat(path.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFDIR)) {
    return true;
  }
  return false;
}

void ConcatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name,
    std::string* path) {
  *path = folder;
  if (path->back() != '/') {
    *path += '/';
  }
  *path = *path + file_name;
}

std::string ConcatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name) {
  std::string path;
  ConcatenateFolderAndFileName(folder, file_name, &path);
  return path;
}

void MakeDir(const std::string& path){
  if(!PathExists(path)){
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }
}

void ReadTxt(const std::string& file_path, 
    std::vector<std::vector<std::string> >& lines, std::string seq){
  if(!FileExists(file_path)){
    std::cout << "file: " << file_path << " dosen't exist" << std::endl;
    exit(0);
  }
  
  std::ifstream infile(file_path, std::ifstream::in);   
  if(!infile.is_open()){
    std::cout << "open file: " << file_path << " failure" << std::endl;
    exit(0);
  }

  std::string line;
  while (getline(infile, line)){ 
    std::string whitespaces(" \t\f\v\n\r");
    std::size_t found = line.find_last_not_of(whitespaces);
    if (found!=std::string::npos){
      line.erase(found+1);

      std::vector<std::string> line_data;
      while (true){
        int index = line.find(seq);
        std::string sub_str = line.substr(0, index);
        if (!sub_str.empty()){
          line_data.push_back(sub_str);
        }

        line.erase(0, index + seq.size());
        if (index == -1){
          break;
        }
      }
      lines.emplace_back(line_data);
    }
    else{
      line.clear();            // str is all whitespace
    }
  }
}

void WriteTxt(const std::string file_path, 
    std::vector<std::vector<std::string> >& lines, std::string seq){
  std::fstream file;
  file.open(file_path.c_str(), std::ios::out|std::ios::app);
  if(!file.good()){
    std::cout << "Error: cannot open file " << file_path << std::endl;
    exit(0);
  }
  for(std::vector<std::string>& line : lines){
    size_t num_in_line = line.size();
    if(num_in_line < 1) continue;
    std::string line_txt = line[0];
    for(size_t i = 1; i < num_in_line; ++i){
      line_txt = line_txt + seq + line[i];
    }
    line_txt += "\n";
    file << line_txt;
  }
  file.close();
}


void ConvertMatToNormalizedArray(cv::Mat& image, Eigen::ArrayXXf& array){
  cv::Mat dst;
  // auto clahe = cv::createCLAHE(2, cv::Size(4, 4));
  // clahe->apply(image, dst);
  dst = image;
  Eigen::MatrixXf matrix;
  cv::cv2eigen(dst, matrix);
  array = matrix.array()/255.0;
}

Eigen::ArrayXXf ConvertMatToArray(const cv::Mat& image){
  Eigen::MatrixXf matrix;
  cv::cv2eigen(image, matrix);
  return matrix.array();
}

cv::Mat ConvertArrayToMat(const Eigen::ArrayXXf& array){
  cv::Mat image;
  Eigen::MatrixXf matrix(array);
  cv::eigen2cv(matrix, image);
  return image;
}

// Eigen pose
Eigen::Vector3d ComputeRelativePose(
    Eigen::Vector3d& pose1, Eigen::Vector3d& pose2){
  Eigen::Vector3d result;
  Eigen::Matrix2d Rw1 = ceres::optimization_2d::RotationMatrix2D(pose1(2));
  result.head(2) = Rw1.transpose() * (pose2.head(2) - pose1.head(2));
  result(2) = pose2(2) - pose1(2);
  result(2) = ceres::optimization_2d::NormalizeAngle(result(2));
  return result;
}

Eigen::Vector3d ComputeAbsolutePose(
  Eigen::Vector3d& pose1, Eigen::Vector3d& relative_pose){
  Eigen::Vector3d result;
  Eigen::Matrix2d Rw1 = ceres::optimization_2d::RotationMatrix2D(pose1(2));
  result.head(2) = pose1.head(2) + Rw1 * relative_pose.head(2);
  result(2) = pose1(2) + relative_pose(2);
  result(2) = ceres::optimization_2d::NormalizeAngle(result(2));
  return result;
}

Eigen::ArrayXXf RotateArray(const Eigen::ArrayXXf& array, float degree)
{
  cv::Mat dst, src=ConvertArrayToMat(array);
  cv::Point2f pc(src.cols/2., src.rows/2.);
  cv::Mat r = cv::getRotationMatrix2D(pc, degree, 1.0);
  cv::warpAffine(src, dst, r, src.size(), cv::INTER_LINEAR, cv::BORDER_WRAP);
  return ConvertMatToArray(dst);
}

Eigen::ArrayXXf WarpArray(const Eigen::ArrayXXf& array, float tx, float ty, float degree)
{
  cv::Mat dst, src=ConvertArrayToMat(array);
  float warp_values[] = {1, 0, tx, 0, 1, ty};
  cv::Mat warp = cv::Mat(2, 3, CV_32F, warp_values);
  cv::warpAffine(src, dst, warp, src.size(), cv::INTER_LINEAR, cv::BORDER_WRAP);
  // ShowArray(ConvertMatToArray(dst), "test", 0);
  return RotateArray(ConvertMatToArray(dst), degree);
}

double NormalizeDegree(double angle_degree) {
  return angle_degree - 360 * floor((angle_degree + 180) / 360);
}


void ShowArray(const Eigen::ArrayXXf& array, std::string window, int waitkey)
{
  auto img = ConvertArrayToMat(array);
  cv::imshow(window, img);
  cv::waitKey(waitkey);
}