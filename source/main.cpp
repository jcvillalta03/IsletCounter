#define PI 3.1415926535897

#include <iostream>
#include <vector>
#include <fstream>
#include <Windows.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <math.h>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>

#include "CTracker.h"
#include "Detector.h"

using namespace cv;
using namespace std;

int maxID = 0;

Point pt1, pt2; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat frame;
int got_roi = 0;

void showMenu()
{
	cout << "\nMenu:\n" << endl;
	cout << "Toggle frame counting with 'f'" << endl;
	cout << "Toggle circle drawing with 'c'" << endl;
	cout << "Toggle line (trace) drawing with 'l'" << endl;
	cout << "Toggle text drawing (Tracker ID) with 't'" << endl;
}

double calcArea(double channelW, double frameW, double frameH)
{
	//channelWidth is equal to 500 microns
	return 500 * (frameW / channelW) * 500 * (frameH / channelW);
}

double calcDiameter(double area)
{
	return 2 * sqrt(area / PI);
}

void createReport(int maxIDNum, vector<double> &areas, double frameW, double frameH, double channelW)
{
	// get date and time
	SYSTEMTIME lpTime;
	GetLocalTime(&lpTime);

	stringstream date;
	date.clear();
	date.str("");
	date << "../objects/reports/report_" << lpTime.wMonth << "-" << lpTime.wDay << "-" << lpTime.wYear << "_" << lpTime.wHour << lpTime.wMinute << lpTime.wSecond << ".csv";
	//date << "reports/report_" << lpTime.wMonth << "-" << lpTime.wDay << "-" << lpTime.wYear << "_" << lpTime.wHour << lpTime.wMinute << lpTime.wSecond << ".txt";
	cout << date.str() << endl;

	//date << "../objects/reports/report_fv_1.csv" << endl;

	//create output file
	ofstream file;
	file.open(date.str(), std::ofstream::out | std::ofstream::app);

	int frameArea = (int) frameW * (int) frameH;
	double realArea = calcArea(channelW, frameW, frameH);
	//vector<int> IEQvector(7,0);
	double IEQ = 0;

	//cout << "Areas" << frameArea << " | " << realArea << endl;

	file << " Found " << maxIDNum << " islets\n" << endl;
	//file << "ID,Area (pixels),Area (microns^2),diameter (microns), % Total"<< endl;
	file << "ID, diameter (microns)" << endl;
	for (int j = 0; j < maxIDNum; j++)
	{
		double percentage = (areas[j+1] / frameArea);
		double AreaSM = percentage * realArea;
		double diameter = calcDiameter(AreaSM);

		if (diameter > 350)
			IEQ += 15.833;
		else if (diameter <= 350 && diameter > 300)
			IEQ += 10.352;
		else if (diameter <= 300 && diameter > 250)
			IEQ += 6.315;
		else if (diameter <= 250 && diameter > 200)
			IEQ += 3.50;
		else if (diameter <= 200 && diameter > 150)
			IEQ += 1.685;
		else if (diameter <= 150 && diameter > 100)
			IEQ += 0.667;
		else if (diameter <= 100 && diameter > 50)
			IEQ += 0.167;

		//file << "ID: " << j+1 << ", A: " << areas[j+1] << " pixels, %: " << percentage << endl;
		//file << j + 1 << "," << areas[j + 1] << "," << AreaSM << "," << diameter <<"," << percentage * 100 << endl;
		file << j + 1 << "," << diameter << endl;

	}
	file << "TotalIEQ: " << IEQ << endl;
	file.close();
	cout << maxID << endl;
	cout << "Report written to " << date.str() << endl;
}

//mouse callback to select ROI in first frame
void onMouseClick(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
		case CV_EVENT_LBUTTONDOWN:
		{
			//if unassigned, assign pt1 as where clicked 
			if (!got_roi)
			{
				pt1.x = x;
				pt1.y = y;
			}
			break;
		}
		case CV_EVENT_LBUTTONUP:
		{
			//if unassigned, assign pt2 as where button release (after click & drag) 
			if (!got_roi)
			{
				pt2.x = x;
				pt2.y = y;

				//draw line and show in frame.
				line(frame, pt1, pt2, Scalar(0,0,255), 2, 8, 0);
				imshow("Frame", frame);

				got_roi = true;
			}
			break;
		}
	}
}

void setMaxID(int ID)
{
	if (ID > maxID)
		maxID = ID;
}

int main(int argc, char** argv)
{
	bool pause = false, showFrameCounter = false;
	bool showCircle = false, showLine = false, showText = false;

	//define colors for tails
	Scalar Colors[]=
	{
		Scalar(255,0,0),
		Scalar(0,255,0),
		Scalar(0,0,255),
		Scalar(255,255,0),
		Scalar(0,255,255),
		Scalar(255,0,255),
		Scalar(255,127,255),
		Scalar(127, 255, 255),
		Scalar(255, 255, 127),
		Scalar(127,127,255),
		Scalar(255, 127, 127),
		Scalar(127, 255, 127),
		Scalar(127,0,127)
	};
	
	string vidFileName;

	//cout << "enter the name of the video file\n" << "eg. IMG_1572.MOV    or ../objects/IMG_1572.MOV\n" << endl;
	//cin >> vidFileName;
	//VideoCapture capture("../objects/validation/Clip2/Clip_2.MOV");
	VideoCapture capture("../objects/Final_Validation/fv_5.mpeg");
	//VideoCapture capture(vidFileName);

	if(!capture.isOpened())
	{
		cout << "Could not open " << vidFileName << endl;
		return 0;
	}

	double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	//Get first frame and select ROI
	capture >> frame;
	line(frame, Point(0, (int)dHeight*.5), Point((int)dWidth, (int)dHeight*.5), Scalar(255.0, 0, 0.0),2);
	imshow("Frame", frame);
	
	cout << "\nDraw a line from the inside edges of the channels. Use green line for reference." << endl;
	setMouseCallback("Frame", onMouseClick, NULL);
	cout << "Was channel properly selected? Y or N. \n If not program will terminate - Please re-run..." << endl;
	
	int ch = waitKey(0);
	
	if ( ch == 'n')
		return -1;
	else if (ch == 'y')
	{
		//continue
	}
	else
		return -1;

	destroyWindow("Frame");
	
	
	double channelWidth = abs(pt1.x - pt2.x);
	cout << "Channel Width: " << channelWidth << " =  500 microns." << endl;

	showMenu();

	// Create the video writer
	VideoWriter out("../objects/capture.mpeg", CV_FOURCC('P', 'I', 'M', '1'), 30, cvSize((int)dWidth, (int)dHeight));
	//VideoWriter out("capture.mpeg", CV_FOURCC('P', 'I', 'M', '1'), 30, cvSize((int)dWidth, (int)dHeight));
	
	// Check if the output video was opened successfully
	if (!out.isOpened())
	{
		cerr << "Could not create video.";
		return -1;
	}

	namedWindow("Video",WINDOW_NORMAL);

	Mat gray;
	Mat back;
	Mat fore;

	float dt = 0.2f;
	float Accel_noise_mag = 1.0;// 0.5;
	double dist_threshold = 100.0;// 60.0;
	int max_skipped_frames = 12;//10;
	int max_trace_length = 10;
	
	CTracker tracker(dt,Accel_noise_mag,dist_threshold,max_skipped_frames,max_trace_length);
	tracker.setFrameDim((int) dHeight, (int) dWidth);
	CDetector* detector = new CDetector((int)dHeight,(int)dWidth);
	
	vector<Point2d> centers;
	vector<double> areas;
	vector<double> reportAreas(10000);

	double numFrames = capture.get(CV_CAP_PROP_FRAME_COUNT);
	for (int fCt = 0; fCt < numFrames - 1; fCt++)	// -1 compensates for first frame we pulled
	{
		if (showFrameCounter)
			cout << "Frame" << fCt << "/" << numFrames << endl;
		capture >> frame;
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		centers = detector->Detect(gray,fore,back,areas);

		if (centers.size()>0)
		{
			tracker.Update(centers,areas);
			for (size_t i = 0; i < tracker.tracks.size(); i++)
			{
				if (tracker.tracks[i]->trace.size() > 1)
				{
					stringstream trackerText;
					trackerText.clear();
					trackerText.str("");
					trackerText << "T # " << tracker.tracks[i]->track_id;

					for (size_t j = 0; j < tracker.tracks[i]->trace.size() - 1; j++)
					{
						if (showLine)
							line(frame, tracker.tracks[i]->trace[j], tracker.tracks[i]->trace[j + 1], Colors[tracker.tracks[i]->track_id % 13], 2, CV_AA);
						if (showCircle)
							circle(frame, tracker.tracks[i]->prediction, (int)sqrt(tracker.tracks[i]->getArea() / PI), Colors[tracker.tracks[i]->track_id % 13], 2, CV_AA);
						if (showText)
						{
							if (fCt % 5)
								putText(frame, trackerText.str(), tracker.tracks[i]->trace[j + 1], CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0), 1);
						}
					}
				}
				setMaxID(tracker.tracks.back()->track_id);
				reportAreas[tracker.tracks[i]->track_id] = tracker.tracks[i]->getArea();
			}
		}

		line(frame, Point(pt1.x, (int)dHeight*.5), Point(pt2.x, (int)dHeight*.5), Scalar(0, 0, 0), 2, 8, 0);
		out << frame;
		//imshow("Video",frame);
		//imshow("gray", gray);
		//imshow("fore", fore);
		//imshow("back", back);

		switch (waitKey(1))
		{

		case 27:	//escape key pressed
			createReport(maxID, reportAreas, dWidth, dHeight, channelWidth);
			return 0;

		case 112: //'p' has been pressed. this will pause/resume the code.
			pause = !pause;
			if (pause == true)
			{
				cout << "Code paused, press 'p' again to resume" << endl;
				while (pause == true)
				{
					//stay in this loop until 
					switch (waitKey())
					{
					//a switch statement inside a switch statement? Mind blown.
					case 112:
						//change pause back to false
						pause = false;
						cout << "Code Resumed" << endl;
						break;
					}
				}
			}
		case 'c':
			showCircle = !showCircle;
			if (showCircle == true)
				cout << "circle drawing enabled..." << endl;
			else
				cout << "circle drawing disabled..." << endl;
			break;
		case 't':
			showText = !showText;
			if (showText == true)
				cout << "writing text enabled..." << endl;
			else
				cout << "writing text disabled..." << endl;
			break;
		case 'l':
			showLine = !showLine;
			if (showLine == true)
				cout << "trace (line) drawing enabled..." << endl;
			else
				cout << "trace (line) drawing disabled..." << endl;
			break;
		case 'f':
			showFrameCounter = !showFrameCounter;
			if (showFrameCounter == true)
				cout << "Frame counter enabled..." << endl;
			else
				cout << "Frame counter disabled..." << endl;
			break;
		}
	}	//end for loop

	delete detector;
	destroyAllWindows();
	createReport(maxID, reportAreas, dWidth, dHeight, channelWidth);
	return 0;
}