#ifndef HELPERS
#define HELPERS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Eigenvalues>
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

#endif
