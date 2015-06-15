#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class CDetector
{
private:
	void DetectContour(Mat& grayImg, Mat& fore, vector<double>& area);

	BackgroundSubtractorMOG2 bgSub;
	vector<Point2d> centers;
	Mat blurred, blobbed;
	int fHeight,fWidth;
public:
	CDetector(int _fHeight, int _fWidth);
	~CDetector(void);
	
	vector<Point2d> Detect(Mat& grayImg, Mat& fore, Mat& back, vector<double>& area);
};

