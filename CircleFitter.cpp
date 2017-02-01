#include "CircleFitter.h" 

using namespace std;


CircleFitter::CircleFitter(){

	srand(time(NULL)); // Seed the time
	RANSACThreshold = 90;
	Iterations = 20;
	ErrorThresh = 1;
	FilterSimilar = true;
}


vector <cv::Point> CircleFitter::ThreeRandomPoints(vector<cv::Point> Contours){

	int NUM;
	int RandIndex[3] = { -1, -1, -1 };
	bool Same;
	vector <cv::Point> CPoints;

	for (int p = 0; p < 3; p++){
		while (true){
			Same = 0;
			NUM = rand() % (Contours.size());

			for (int a = 0; a < 3; a++){
				if (RandIndex[p] == NUM){
					Same = 1;
				}//end if num already chosen 
			}//end for a
			if (!Same){
				RandIndex[p] = NUM;
				CPoints.push_back(Contours[NUM]);
				break;
			}//end if number is not already chosen
		}//end while true
	}//end for 3
	return CPoints;

}

pair<cv::Point, double> CircleFitter::CircleFromPoints(cv::Point p1, cv::Point p2, cv::Point p3){

	double offset = pow(p2.x, 2) + pow(p2.y, 2);
	double bc = (pow(p1.x, 2) + pow(p1.y, 2) - offset) / 2.0;
	double cd = (offset - pow(p3.x, 2) - pow(p3.y, 2)) / 2.0;
	double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x)* (p1.y - p2.y);
	double TOL = 0.0000001;

	if (abs(det) < TOL) { return make_pair(cv::Point(0, 0), 0); }

	double idet = 1 / det;
	double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
	double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
	double radius = sqrt(pow(p2.x - centerx, 2) + pow(p2.y - centery, 2));

	return make_pair(cv::Point(centerx, centery), radius);

}//end circle from points

double CircleFitter::dist(cv::Point x, cv::Point y){

	return sqrt((x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y));

}//end dist

pair<cv::Point, double> CircleFitter::FitCircle(vector<cv::Point> Contours){

	int Matches = 0;
	int BestMatch = 0;
	double PointDist,PointErr;
	vector <cv::Point> ModelPoints;
	pair<cv::Point, double> ModelCircle, BestCircle;

	if (Contours.size() > 4){
		for (int i = 0; i < Iterations; i++){
			Matches = 0;
			ModelPoints = ThreeRandomPoints(Contours);
			ModelCircle = CircleFromPoints(ModelPoints[0], ModelPoints[1], ModelPoints[2]);
			if (ModelCircle.second>0){
				for (int p = 0; p < Contours.size(); p++){
					PointDist = dist(Contours[p], ModelCircle.first);
					PointErr = abs(PointDist - ModelCircle.second);

					if (PointErr < ErrorThresh){
						Matches++;
					}//end if circle match

				}//end for p iterations
			}//end if Circle found

			if (Matches > BestMatch){
				BestMatch = Matches;
				BestCircle = ModelCircle;
			}//end if best matches
		}//end for i iterations 
	}//end if Contours >6
	if ((float(BestMatch) / float(Iterations)) * 100>RANSACThreshold){

		return BestCircle;

	}
	else{
		return make_pair(cv::Point(0, 0), -1);
	}
}//end circle fitter


pair<cv::Point, double> CircleFitter::FitCircle(vector<cv::Point> Contours, int MinR, int MaxR){

	int Matches = 0;
	int BestMatch = 0;
	double PointDist, PointErr;
	vector <cv::Point> ModelPoints, TestPoints;
	pair<cv::Point, double> ModelCircle, TestCircle, BestCircle;

	if (Contours.size() > 4){
		for (int i = 0; i < Iterations; i++){
			Matches = 0;
			ModelPoints = ThreeRandomPoints(Contours);
			ModelCircle = CircleFromPoints(ModelPoints[0], ModelPoints[1], ModelPoints[2]);
			if ((ModelCircle.second>0) && (ModelCircle.second >= MinR) && (ModelCircle.second <= MaxR))
			{
				for (int p = 0; p < Contours.size(); p++){
					PointDist = dist(Contours[p], ModelCircle.first);
					PointErr = abs(PointDist - ModelCircle.second);

					if (PointErr < ErrorThresh){
						Matches++;
					}//end if circle match

				}//end for p iterations

				if (Matches > BestMatch){
					BestMatch = Matches;
					BestCircle = ModelCircle;
				}//end if best matches
			}
		}//end for i iterations 
	}//end if Contours >6
	if ((float(BestMatch) / float(Iterations)) * 100>RANSACThreshold){

		return BestCircle;

	}
	else{
		return make_pair(cv::Point(0, 0), -1);
	}
}//end circle fitter


pair<cv::Point, double> CircleFitter::FitCircle(vector<cv::Point> Contours, int MinR, int MaxR, int Iteration){


	int Matches = 0;
	int BestMatch = 0;
	double PointDist, PointErr;
	vector <cv::Point> ModelPoints, TestPoints;
	pair<cv::Point, double> ModelCircle, TestCircle, BestCircle;

	if (Contours.size() > 4){
		for (int i = 0; i < Iteration; i++){
			Matches = 0;
			ModelPoints = ThreeRandomPoints(Contours);
			ModelCircle = CircleFromPoints(ModelPoints[0], ModelPoints[1], ModelPoints[2]);
			if ((ModelCircle.second>0) && (ModelCircle.second >= MinR) && (ModelCircle.second <= MaxR))
			{
				for (int p = 0; p < Contours.size(); p++){
					PointDist = dist(Contours[p], ModelCircle.first);
					PointErr = abs(PointDist - ModelCircle.second);

					if (PointErr < ErrorThresh){
						Matches++;
					}//end if circle match

				}//end for p iterations

				if (Matches > BestMatch){
					BestMatch = Matches;
					BestCircle = ModelCircle;
				}//end if best matches
			}
		}//end for i iterations 
	}//end if Contours >6
	if ((float(BestMatch) / float(Iteration)) * 100>RANSACThreshold){
		return BestCircle;
	}
	else{
		return make_pair(cv::Point(0, 0), -1);
	}
}//end circle fitter

vector<pair<cv::Point, double>> CircleFitter::FindCircles(cv::Mat Frame, int ThresholdValue, int MinRadius, int MaxRadius, int Iteration, int MinContourArea, int MaxContourArea){

	cv::Mat ThresholdedImage;

	vector<vector<cv::Point> > Contours;

	bool SameCirc = 0;

	pair<cv::Point, double> Circle;
	vector <pair<cv::Point, double>> CircleFound;

	threshold(Frame, ThresholdedImage, ThresholdValue, 255, cv::THRESH_BINARY_INV); // Threshold to create input
	imshow("thresh", ThresholdedImage);
	//Find the Contours in the image
	findContours(ThresholdedImage, Contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (int i = 0; i < Contours.size(); i++){
		if ((contourArea(Contours[i]) >= MinContourArea) && (contourArea(Contours[i]) <= MaxContourArea)){
			SameCirc = 0;
			Circle = FitCircle(Contours[i], MinRadius, MaxRadius, Iteration);

			if (Circle.second > 0){
				if (FilterSimilar){
					if (CircleFound.size() > 0){
						for (int p = 0; p < CircleFound.size(); p++){
							double CircDist = dist(CircleFound[p].first, Circle.first);
							if (CircDist < (MinRadius / 2)){
								SameCirc = 1;
							}
						}
					}

					if (!SameCirc){
						CircleFound.push_back(Circle);
					}
				}
				else{
					CircleFound.push_back(Circle);
				}
			}
		}
	}
	return CircleFound;

}
	

