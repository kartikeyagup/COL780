#ifndef HELPERS_H
#define HELPERS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

void translateImg(
  cv::Mat &img, 
  cv::Mat&img_out, 
  double offsetx, 
  double offsety);

cv::Mat ImageStabilisation(
  cv::Mat previous_frame,
  cv::Mat present_frame,
  std::vector<cv::Point2f> previous_points,
  std::vector<cv::Point2f> present_points,
  std::vector<uchar> status);

bool IsValidFeature(double Ixx, double Iyy, double Ixy);

void myGoodFeaturesToTrack(
  cv::Mat &image,
  cv::Mat &gradx,
  cv::Mat &grady,
  std::vector<cv::Point2f> &features);

cv::Point2f WarpPoint(float point_x, float point_y, 
  Eigen::Matrix<float, 2, 3> &warp_matrix);

bool TrackRegion(cv::Mat &image_initial,
  cv::Mat &gradx,
  cv::Mat &grady,
  cv::Mat &image_final,
  cv::Point2f &tracked_in,
  cv::Point2f &tracked_out,
  int gridsize);

void myOpticalFlow(cv::Mat &image_initial,
  cv::Mat &image_final,
  std::vector<cv::Point2f> tracked_in,
  std::vector<cv::Point2f> tracked_out,
  int gridsize,
  std::vector<uchar> tracking_status);

#endif
