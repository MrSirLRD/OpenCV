#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include "CircleFitter.h" 

using namespace std;
using namespace cv;

int main(){


	//Open Video Stream
	VideoCapture vcap1(0);
	if (!vcap1.isOpened()){
		cout << "Error opening video stream or file" << endl;
		waitKey(0);
		return -1;
	}
	CircleFitter CircleFit;
	Mat RawFrame,FrameGreyScale;
	CircleFit.RANSACThreshold=95;
	for (;;){
		vector<pair<cv::Point, double>> Circles;
		//Get the frame
		vcap1 >> RawFrame;
		cvtColor(RawFrame, FrameGreyScale, CV_RGB2GRAY);

		Circles = CircleFit.FindCircles(FrameGreyScale, 150, 1, 30, 50, 100, 300000);
		for (int i = 0; i < Circles.size(); i++){
			cv::circle(RawFrame, Circles[i].first, Circles[i].second,Scalar(255,0,0),4,8);
		}
		imshow("Output Window", RawFrame);
		waitKey(1);
	}//end for(;;)


	return 0;
}
