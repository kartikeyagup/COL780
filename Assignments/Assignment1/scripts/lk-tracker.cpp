#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main() {
	cv::VideoCapture cap;
	cap.open(0);
	cv::Mat present_frame;
	cv::namedWindow("Tracking", 1);

	if (!cap.isOpened()) {
		std::cout << "Unable to start camera stream\n";
		return 1;
	}

	while (1) {
		cap >> present_frame;
		cv::imshow("Tracking", present_frame);
		
		char c = (char)cv::waitKey(10);
		if ( c== 27 ) 
			break;
	}

	return 0;
}