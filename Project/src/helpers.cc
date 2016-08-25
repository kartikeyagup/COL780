#include "helpers.h"

camera_frame createCameraFrame(cv::Mat &frame,
  Eigen::Matrix3d& rotation,
  Eigen::Vector3d& translation,
  int cam_id,
  camera_description& camera_properties) {
  camera_frame result;
  result.frame = frame;
  result.rotation = rotation;
  result.translation = translation;
  result.cam_id = cam_id;
  result.camera_properties = camera_properties;
  return result;
}

std::vector<double> ParseInput(std::string filename) {
  std::vector<double> answer;
  return answer;
} 

void LoadIntrinsics(std::string intrinsics,
  calibration& calibration) {
  std::vector<double> values = ParseInput(intrinsics);
  assert(values.size() == 4);
  calibration.fx = values[0];
  calibration.fy = values[1];
  calibration.cx = values[2];
  calibration.cy = values[3];
}

void LoadExtrinsics(std::string extrinsics,
  Eigen::Matrix3d& rotation,
  Eigen::Vector3d& translation) {
  std::vector<double> values = ParseInput(extrinsics);
  assert(values.size() == 12);
  rotation << values[0], values[1], values[2],
              values[3], values[4], values[5],
              values[6], values[7], values[8];
  translation << values[9], values[10], values[11];
}

camera_frame LoadCameraFrameFromDisk(std::string image_path,
  std::string intrinsics, std::string extrinsics) {
  camera_frame result;
  result.frame = cv::imread(image_path);
  LoadIntrinsics(intrinsics, result.camera_properties.camera_calibration);
  LoadExtrinsics(extrinsics, result.rotation, result.translation);
  result.camera_properties.image_dimension.sx = result.frame.cols;
  result.camera_properties.image_dimension.sy = result.frame.rows;
  return result;
}
