#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CTrack
{
public:
	vector<Point2d> trace;
	Rect bounding_rect;
	static size_t NextTrackID;
	size_t track_id;
	size_t skipped_frames; 
	Point2d prediction;
	TKalmanFilter* KF;
	
	double avgArea;			//average area to be calculated with area vector
	vector<double> area;

	CTrack(Point2f p, float dt, float Accel_noise_mag);
	void setArea(double currArea);
	void calcArea();
	double getArea();
	void printArea();


	~CTrack();
};


class CTracker
{
public:
	
	float dt; 
	float Accel_noise_mag;
	double dist_thres;
	int maximum_allowed_skipped_frames;
	int max_trace_length;
	int fWidth, fHeight;


	vector<CTrack*> tracks;
	void setFrameDim(int _fHeight, int _fWidth);
	void Update(vector<Point2d>& detections, vector<double>& areaVec);

	CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~CTracker(void);
};

