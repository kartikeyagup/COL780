#include "helpers.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

const int kMaxCount = 1000;

int main() {
  cv::VideoCapture cap;
  cap.open(0);
  cv::Mat present_frame, previous_gray, present_gray;
  std::vector<cv::Point2f> tracking_points, new_positions;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 
    20, 0.03);
  cv::Size subPixWinSize(10, 10), window_size(31, 31);
  int frameid=0;
  cv::namedWindow("Tracking", 1);
  
  if (!cap.isOpened()) {
    std::cerr << "Unable to start camera stream\n";
    return 1;
  }

  while (1) {
    cap >> present_frame;
    cv::cvtColor(present_frame, present_gray, cv::COLOR_BGR2GRAY);
    if (frameid==0) {
      present_gray.copyTo(previous_gray);
    }
    if (frameid%50==0) {
      // Add more features to track
      cv::goodFeaturesToTrack(present_gray, tracking_points, kMaxCount, 0.01, 
        10, cv::Mat(), 3, 0, 0.04);
      if (tracking_points.size() > 0) {
        cv::cornerSubPix(present_gray, tracking_points, subPixWinSize, 
          cv::Size(-1,-1), termcrit);
      }
      std::cerr << "Got features of size " << tracking_points.size() <<"\n"; 
    }

    if (tracking_points.size() > 0 ) {
      // Do tracking
      std::vector<uchar> tracking_status; 
      std::vector<float> tracking_error;
      if (previous_gray.empty()) {
        std::cerr << "Image was empty\n";
      }
      calcOpticalFlowPyrLK(previous_gray, 
        present_gray, 
        tracking_points, 
        new_positions, 
        tracking_status, 
        tracking_error,
        window_size, 
        3, 
        termcrit, 
        0, 
        0.001);
      std::vector<cv::Point2f> updated_points;
      present_frame = ImageStabilisation(previous_gray, present_gray, tracking_points, new_positions, tracking_status);
      for (int i=0; i<new_positions.size() ; i++) {
        if (tracking_status[i]) {
          updated_points.push_back(new_positions[i]);
          cv::circle(present_frame, new_positions[i], 3, cv::Scalar(0, 255, 0), -1, 8);         
        }
      }
      tracking_points = updated_points;
    }
    cv::imshow("Tracking", present_frame);
    
    char c = (char)cv::waitKey(10);
    if ( c== 27 ) 
      break;
    present_gray.copyTo(previous_gray);
    frameid=frameid+1;
  }
  return 0;
}
