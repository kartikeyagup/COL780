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

bool IsValidFeature(double Ixx, double Iyy, double Ixy) {
  double a_c2 = (Ixx - Iyy)*(Ixx - Iyy);
  double b_2 = Ixy*Ixy;
  double apc = Ixx + Iyy;
  double e1 = apc + sqrt(a_c2 + 4*b_2);
  double e2 = apc - sqrt(a_c2 + 4*b_2);
  if ((e1 > 500000) && (e2 > 500000)) {
    return true;
  }
  return false;
}

void myGoodFeaturesToTrack(cv::Mat &image, cv::Mat &gradx, cv::Mat &grady, std::vector<cv::Point2f> &features) {
  cv::Mat g1, g2;
  features.clear();
  Sobel(image, g1, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
  Sobel(image, g2, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
  convertScaleAbs(g1, gradx);
  convertScaleAbs(g2, grady);
  int gridsize = 7;
  for (int j=0; j < image.rows-gridsize; j++) {
    double Ixx=0, Ixy=0, Iyy=0;
    double xval, yval;
    for (int j1=j; j1<j+gridsize; j1++) {
      for (int i1=0; i1<gridsize; i1++) {
        xval = gradx.at<uchar>(j1, i1);
        yval = grady.at<uchar>(j1, i1);
        Ixx += xval*xval;
        Ixy += xval*yval;
        Iyy += yval*yval;
      }
    }
    for (int i=0; i < image.cols-gridsize; i++) {
      if (i>0) {
        for (int k=0; k<gridsize; k++) {
          xval = gradx.at<uchar>(j+k, i-1);
          yval = grady.at<uchar>(j+k, i-1);
          Ixx -= xval*xval;
          Ixy -= xval*yval;
          Iyy -= yval*yval;
        }
        for (int k=0; k<gridsize; k++) {
          xval = gradx.at<uchar>(j+k, i+gridsize);
          yval = grady.at<uchar>(j+k, i+gridsize);
          Ixx += xval*xval;
          Ixy += xval*yval;
          Iyy += yval*yval;
        }
      }
      if (IsValidFeature(Ixx, Iyy, Ixy)) {
        features.push_back(cv::Point2f(i+gridsize/2, j+gridsize/2));
      }
    }
  }
}
