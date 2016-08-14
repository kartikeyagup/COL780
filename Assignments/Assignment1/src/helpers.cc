#include "helpers.h"

void translateImg(cv::Mat &img, cv::Mat&img_out, 
    double offsetx, double offsety) {
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
  cv::warpAffine(img, img_out, trans_mat, img.size());
}

cv::Mat ImageStabilisation(
  cv::Mat previous_frame,
  cv::Mat present_frame,
  std::vector<cv::Point2f> previous_points,
  std::vector<cv::Point2f> present_points,
  std::vector<uchar> status) {
  cv::Point2f total_deviation;
  int num_points = 0;
  for (int i=0; i< previous_points.size() ; i++) {
    if (status[i]) {
      num_points +=1;
      total_deviation += present_points[i];
      total_deviation -= previous_points[i]; 
    }
  }
  if (num_points>0) {
    cv::Mat image_out;
    present_frame.copyTo(image_out);
    total_deviation = total_deviation/num_points;
    std::cerr << "Deviation: " << total_deviation.x 
              << "\t" << total_deviation.y <<"\n" ;
    translateImg(present_frame, image_out, -total_deviation.x, -total_deviation.y);
    return image_out;
  }
  else {
    return present_frame;   
  }
}
