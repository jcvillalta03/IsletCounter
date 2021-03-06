#include "opencv2/opencv.hpp"
#include <opencv/cv.h>

using namespace cv;
using namespace std;

// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/

class TKalmanFilter
{
public:
	//constructor and destructor
	TKalmanFilter(Point2f p,float dt=0.2,float Accel_noise_mag=0.5);
	~TKalmanFilter();
	
	KalmanFilter* kalman;
	double deltatime; 
	Point2f LastResult;
	
	Point2f GetPrediction();
	Point2f Update(Point2f p, bool DataCorrect);
};

