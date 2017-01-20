#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <sys/timeb.h>
#include <time.h>

using namespace cv;
using namespace std;

int getMilliCount(){
	timeb tb;
	ftime(&tb);
	int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
	return nCount;
}

int getMilliSpan(int nTimeStart){
	int nSpan = getMilliCount() - nTimeStart;
	if (nSpan < 0)
		nSpan += 0x100000 * 1000;
	return nSpan;
}


vector <Point> ThreeRandomPoints(vector<Point> contours){
	int NUM;
	int RandIndex[3] = { -1, -1, -1 };
	bool Same;
	vector <Point> CPoints;

	for (int p = 0; p < 3; p++){
		while (true){
			Same = 0;

			NUM = rand() % (contours.size());

			for (int a = 0; a < 3; a++){
				if (RandIndex[p] == NUM){
					Same = 1;
				}
			}
			if (!Same){
				RandIndex[p] = NUM;
				CPoints.push_back(contours[NUM]);
				break;
			}
		}
	}
	return CPoints;
}

double dist(Point x, Point y)
{
	return sqrt((x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y));
}

pair<Point, double> circleFromPoints(Point p1, Point p2, Point p3)
{
	double offset = pow(p2.x, 2) + pow(p2.y, 2);
	double bc = (pow(p1.x, 2) + pow(p1.y, 2) - offset) / 2.0;
	double cd = (offset - pow(p3.x, 2) - pow(p3.y, 2)) / 2.0;
	double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y);
	double TOL = 0.0000001;
	if (abs(det) < TOL) { return make_pair(Point(0, 0), 0); }

	double idet = 1 / det;
	double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt(pow(p2.x - centerx, 2) + pow(p2.y - centery, 2));

	return make_pair(Point(centerx, centery), radius);
}



pair<Point, double> FitCircle(vector<Point> contours,int MinR,int MaxR){
	int iterations = 20;
	int CenterThresh=10;
	int RadiusThresh=10;

	int Matches=0;
	int BestMatch = 0;
	double CenterDist, RadiusDiff;
	vector <Point> ModelPoints, TestPoints;
	pair<Point, double> ModelCircle, TestCircle, BestCircle;

	if (contours.size() > 6){
		for (int i = 0; i < iterations; i++){
			Matches = 0;
			ModelPoints = ThreeRandomPoints(contours);
			ModelCircle = circleFromPoints(ModelPoints[0], ModelPoints[1], ModelPoints[2]);
			if ((ModelCircle.second >= MinR) && (ModelCircle.second <= MaxR))
			{
				for (int p = 0; p < iterations; p++){
					TestPoints = ThreeRandomPoints(contours);
					TestCircle = circleFromPoints(TestPoints[0], TestPoints[1], TestPoints[2]);

					CenterDist = dist(TestCircle.first, ModelCircle.first);
					RadiusDiff = sqrt(pow(TestCircle.second - ModelCircle.second, 2));
					if ((CenterDist < CenterThresh) && (RadiusDiff < RadiusThresh)){
						Matches++;
					}

				}

				if (Matches > BestMatch){
					BestMatch = Matches;
					BestCircle = ModelCircle;
				}
			}
		}
	}
	//cout << (float(BestMatch) / float(iterations))*100 << endl;
	if ((float(BestMatch) / float(iterations)) * 100>50){

		return BestCircle;

	}
	else{
		return make_pair(Point(0, 0), -1);
	}
}




int main(int argc, char *argv[])
{
	srand(time(NULL)); // Seed the time
	namedWindow("OuPut", 1);

	Mat frame, frameRaw, Edges;
	pair<Point, double> Circle;
	int erosion_size = 1;
	int erosion_type = MORPH_ELLIPSE;
	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	int start = getMilliCount();

	const string inputSettingsFile = "out_camera_data.xml";
	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
	}
	Mat DS, CM;
	fs["Distortion_Coefficients"] >> DS;
	fs["Camera_Matrix"] >> CM;
	fs.release();

	Mat raw_frame,UnWarpedRaw;
	VideoCapture cap(0);
	int Thresh;
	createTrackbar("thresh", "OuPut", &Thresh, 255);

	bool SameCirc;
	for (;;)
	{
		int milliSecondsElapsed = getMilliSpan(start);
		start = getMilliCount();
		vector<vector<Point> > contours;
		vector <Point> CircleCenters;
		//Get the frame
		cap >> raw_frame;
		undistort(raw_frame, UnWarpedRaw, CM, DS);

		cvtColor(UnWarpedRaw, frame, CV_RGB2GRAY);
		//frameRaw = imread("Circles.jpg",1);
	//	int ScaleDown = 5;
	//	resize(frameRaw, frameRaw, Size(frameRaw.cols / ScaleDown, frameRaw.rows / ScaleDown), 0, 0, INTER_LINEAR);

	//	cvtColor(frameRaw, frame, CV_RGB2GRAY9
	//	GaussianBlur(frame, frame, Size(9, 9), 0, 0);
		//Canny(frame,Edges,150,250,5,1);

		threshold(frame, Edges, Thresh, 255, THRESH_BINARY_INV); // Threshold to create input
		//threshold(frame, Edges, 20, 255, THRESH_BINARY_INV); // Threshold to create input
		imshow("Edges", Edges);

		//dilate(Edges, Edges, element);

		//Find the contours in the foreground
		findContours(Edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		//cout << contours.size() << endl;

		for (int i = 0; i < contours.size(); i++){
			if ((contourArea(contours[i]) >= 300) && (contourArea(contours[i]) <= 3000))
			{
				//	drawContours(frame, contours, i, Scalar(0), 1, 8);
				//putText(frame, s, Circle.first, 0, 0.8, Scalar(0, 255, 0), 2, 8);
				SameCirc = 0;
				//cout << "hello" << endl;
				Circle = FitCircle(contours[i],23,27);
				if (Circle.second > 0){
					if (CircleCenters.size() > 0){
						for (int p = 0; p < CircleCenters.size(); p++){
							double CircDist = dist(CircleCenters[p], Circle.first);
							if (CircDist < 15){
								SameCirc = 1;
							}
						}

					}

					if (!SameCirc){
						circle(UnWarpedRaw, Circle.first, Circle.second, Scalar(255,0,0), 4, 8);
						CircleCenters.push_back(Circle.first);
						Rect r(cvPoint(Circle.first.x - Circle.second, Circle.first.y - Circle.second), Size(2 * Circle.second + 5, 2 * Circle.second + 5));
						if ((r.tl().x > 0) && (r.tl().y > 0) && (r.br().x < frame.cols) && (r.br().y < frame.rows)){

							Mat1b mask(frame.size(), uchar(0));
							circle(mask, Circle.first, Circle.second, Scalar(255), CV_FILLED);
							Mat res(frame.size(), CV_8UC3, Scalar(255, 0, 0));
							threshold(frame, frame, 100, 255, THRESH_BINARY_INV); // Threshold to create input

							frame.copyTo(res, mask);

							Mat Boardered;
							Boardered = res(r);
							//frame(r).copyTo(Boardered);
							resize(Boardered, Boardered, Size(50, 50), 0, 0);
							//imshow("circle", Boardered);
						}
					}
				}
			}
		}
	//	cout <<"end"<< endl;

		imshow("OuPut", UnWarpedRaw);
		cout << 1000 / (milliSecondsElapsed + 1) << endl;
		waitKey(1);
	}
	return 0;
}