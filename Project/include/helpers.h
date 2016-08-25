#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct image_size {
  int sx;
  int sy;
};

struct calibration {
  float fx;
  float fy;
  float cx;
  float cy;
};

struct camera_description {
  image_size image_dimension;
  calibration camera_calibration;
};

struct camera_frame {
  cv::Mat frame;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  int cam_id;
  camera_description camera_properties;
};

camera_frame createCameraFrame(cv::Mat &frame,
  Eigen::Matrix3d& rotation,
  Eigen::Vector3d& translation,
  int cam_id,
  camera_description& camera_properties);

std::vector<double> ParseInput(std::string filename);

void LoadExtrinsics(std::string extrinsics, 
  Eigen::Matrix3d& rotarion,
  Eigen::Vector3d& translation);

void LoadIntrinsics(std::string intrinsics,
  calibration& calibration);

camera_frame LoadCameraFrameFromDisk(std::string image_path, 
  std::string intrinsics, 
  std::string extrinsics);

#endif
