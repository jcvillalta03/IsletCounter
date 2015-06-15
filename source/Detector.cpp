#include "Detector.h"

using namespace cv;
using namespace std;

CDetector::CDetector(int _fHeight, int _fWidth)
{
	fHeight = _fHeight;
	fWidth = _fWidth;
	bgSub.set("nmixtures", 4);
	bgSub.set("detectShadows", false);
}

void CDetector::DetectContour(Mat& grayImg, Mat& fore, vector<double>& areaVec)
{
	centers.clear();
	areaVec.clear();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	findContours(fore, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//, Point(0, 0));

	if (contours.size()>0)
	{
		/// Get the moments
		vector<Moments> mu(contours.size());
		for (size_t i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		///  Get the mass centers:
		vector<Point2d> mc(contours.size());
		for (size_t i = 0; i < contours.size(); i++)
		{
			//accurate way to get mass center (centroid)
			mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

			//define minimum area and maximum area
			double minArea = 0.0003 * fWidth * fHeight;
			double maxArea = 0.90 * fWidth * fHeight;

			if ((mc[i].y >(fHeight*.15) && mc[i].y < (fHeight*.85)) && contourArea(contours[i]) > minArea)
			{
				centers.push_back(mc[i]);
				areaVec.push_back(contourArea(contours[i]));

			}
			else
			{
				contours.erase(contours.begin() + i);
			}
		}
	}
	cv::drawContours(grayImg, contours, -1, cv::Scalar(0, 0, 0), CV_FILLED, CV_AA);
}
vector<Point2d> CDetector::Detect(Mat& grayImg, Mat& fore, Mat& back, vector<double>& areaVec)
{
		cv::blur(grayImg, blurred, cv::Size(10, 10));
		bgSub.operator ()(blurred, fore);
		bgSub.getBackgroundImage(back);
		cv::blur(fore, fore, cv::Size(10, 10));
		cv::erode(fore, fore, Mat());
		//imshow("b",blobbed);

		cv::dilate(fore, fore, cv::Mat());
		cv::threshold(fore, fore, 30, 255, CV_THRESH_BINARY);

		// rects - bounding rectangles
		// centers - centers of bounding rectangles
		/*
		Mat fg2;
		fg.convertTo(fg2,CV_32FC1);
		cv::GaussianBlur(fg2,fg2,Size(5,5),1.0);
		cv::Laplacian(fg2,fg2,CV_32FC1);

	
		fg2.convertTo(fg2,CV_8UC1);
		cv::applyColorMap(fg2,fg2,COLORMAP_JET);
		imshow("Foreground",fg2);
		*/
		//DetectContour(grayImg, fore, centers, areaVec);
		DetectContour(grayImg, fore, areaVec);
		return centers;
}

CDetector::~CDetector(void)
{
	//delete bs;

}