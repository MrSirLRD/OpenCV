
#ifndef CIRCLEFITTER
#define CIRCLEFITTER

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

class CircleFitter{

private:
	vector <cv::Point> ThreeRandomPoints(vector<cv::Point> contours);
	pair<cv::Point, double> CircleFromPoints(cv::Point p1, cv::Point p2, cv::Point p3);
	double dist(cv::Point x, cv::Point y);

public:

	CircleFitter();

	int RANSACThreshold;//Consensus of model in percent
	int Iterations, CenterThresh, RadiusThresh;

	bool FilterSimilar; //Set to true to filter circles with a similar center, used in FindCircles - set to true by default

	pair<cv::Point, double> FitCircle(vector<cv::Point> contours);//Function Will try to fit a circle to a set of Points using basic RANSAC 

	pair<cv::Point, double> FitCircle(vector<cv::Point> contours, int MinR, int MaxR);//Function Will try to fit a circle to a set of Points using basic RANSAC, it will also filter a range of circle radius. 

	pair<cv::Point, double> FitCircle(vector<cv::Point> contours, int MinR, int MaxR, int Iteration);//Function Will try to fit a circle to a set of Points using basic RANSAC input includes the number of iterations performed in both the number of models tested and number of consensus tests, it will also filter a range of circle radius. 

	vector<pair<cv::Point, double>> FindCircles(cv::Mat Frame, int ThresholdValue, int MinRadius, int MaxRadius, int Iteration, int MinContourArea, int MaxContourArea);//An example Function that will find circles in a greyscale frame, includes input for radius and contour sizes as well as RANSAC iterations
};


#endif