#include "Ctracker.h"
using namespace cv;
using namespace std;

size_t CTrack::NextTrackID=0;

// Track constructor.
CTrack::CTrack(Point2f pt, float dt, float Accel_noise_mag)
{
	track_id = NextTrackID;		// give trackID the next available
	NextTrackID++;				// increment the ID.
	KF = new TKalmanFilter(pt, dt, Accel_noise_mag);	// every track needs a Kalman Filter
	prediction = pt;	//coordinates of stored pts used for prediction
	skipped_frames = 0;
	calcArea();
}

//add area to area vector to calc area
void CTrack::setArea(double currArea)
{
	area.push_back(currArea);
	calcArea();
}

//function to calculate area from the area vector
void CTrack::calcArea()
{
	if (area.size() == 0)
	{
		avgArea = -1;
	}
	else
	{
		if (area.size() > 3)
		{
			if (((area.back() > (1.5 * avgArea)) || (area.back() < (.66 * avgArea))))
			{
				area.pop_back();
			}
		}
		double sum = 0;
		int numElem = area.size();
		for (int i = 0; i < numElem; i++)
			sum += area.at(i);
		avgArea = (sum / numElem);
	}
}

//function to return area 
double CTrack::getArea()
{
	return avgArea;
}

void CTrack::printArea()
{
	printf("ID: %d,A: %f\n", track_id, avgArea);
}

// Track Destructor 
CTrack::~CTrack()
{
	// Free resources.
	delete KF;
	area.clear();
}

//****** Tracker Class *****//

//Tracker Constructor
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames, int _max_trace_length)
{
	dt = _dt;
	Accel_noise_mag = _Accel_noise_mag;
	dist_thres = _dist_thres;
	maximum_allowed_skipped_frames = _maximum_allowed_skipped_frames;
	max_trace_length = _max_trace_length;
}

void CTracker::setFrameDim(int _fHeight, int _fWidth)
{
	fHeight = _fHeight;
	fWidth = _fWidth;
}

//Update tracker from centroids found
void CTracker::Update(vector<Point2d>& detections, vector<double>& areaVec)
{
	// if no tracks exist, each detection (center) is a new track 
	if (tracks.size() == 0)
	{
		for (size_t i = 0; i<detections.size(); i++)
		{
			CTrack* tr = new CTrack(detections[i], dt, Accel_noise_mag);
			tr->setArea(areaVec[i]);
			
			tracks.push_back(tr);
		}
	}

	int T = tracks.size();		// треки
	int D = detections.size();	// детекты

	vector< vector<double> > Cost(T, vector<double>(D));
	vector<int> assignment; // назначения

	double dist;
	for (size_t i = 0; i<tracks.size(); i++)
	{
		//Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for (size_t j = 0; j<detections.size(); j++)
		{
			Point2d diff = (tracks[i]->prediction - detections[j]);
			dist = sqrt(diff.x*diff.x + diff.y*diff.y);
			Cost[i][j] = dist;
		}
	}

	// Solving assignment problem (tracks and predictions of Kalman filter)

	AssignmentProblemSolver APS;
	APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

	// clean assignment from pairs with large distance

	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for (size_t i = 0; i<assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			if (Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i] = -1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{
			// If track doesn't have an assigned detection, then increment skipped frames counter.
			tracks[i]->skipped_frames++;
		}

	}

	// If a track didn't get a new detection for a long time, remove the track.
	for (size_t i = 0; i<tracks.size(); i++)
	{
		if (tracks[i]->skipped_frames >(size_t) maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin() + i);
			assignment.erase(assignment.begin() + i);
			i--;
		}
	}
	/*
	// If a track didn't get a new detection for a long time, remove the track.
	for (size_t i = 0; i<tracks.size(); i++)
	{
		if (tracks[i]->skipped_frames >(size_t) maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin() + i);
			assignment.erase(assignment.begin() + i);
			i--;
		}
	}
	*/
	// find detections that have not been assigned
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for (size_t i = 0; i<detections.size(); i++)
	{
		it = find(assignment.begin(), assignment.end(), i);
		if (it == assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	//then start new tracks for them
	if (not_assigned_detections.size() != 0)
	{
		for (size_t i = 0; i<not_assigned_detections.size(); i++)
		{
			if (detections[not_assigned_detections[i]].y > .5 * fHeight)
			{
				CTrack* tr = new CTrack(detections[not_assigned_detections[i]], dt, Accel_noise_mag);
				tr->setArea(areaVec[not_assigned_detections[i]]);
				//areaVec.erase(areaVec.begin()+not_assigned_detections[i]);
				tracks.push_back(tr);
			}
		}
	}

	// Update the state of the Kalman Filters 
	for (size_t i = 0; i<assignment.size(); i++)
	{
		if ((tracks[i]->area.size() < 3))
		{
			// If track updated less than one time, than filter state is not correct.
			tracks[i]->KF->GetPrediction();

			if (assignment[i] != -1) // If we have assigned the detection, then update its coordinates,
			{
				tracks[i]->skipped_frames = 0;
				tracks[i]->prediction = tracks[i]->KF->Update(detections[assignment[i]], 1);
				tracks[i]->setArea(areaVec[assignment[i]]);
			}
			else				  // if not continue using predictions
			{
				tracks[i]->prediction = tracks[i]->KF->Update(Point2f(0, 0), 0);
			}

			if (tracks[i]->trace.size() > (size_t)max_trace_length)
			{
				tracks[i]->trace.erase(tracks[i]->trace.begin(), tracks[i]->trace.end() - max_trace_length);
			}
			tracks[i]->trace.push_back(tracks[i]->prediction);
			tracks[i]->KF->LastResult = tracks[i]->prediction;
		}
		else
		{
			if ((areaVec[assignment[i]] < (1.5 * tracks[i]->avgArea)) || (areaVec[assignment[i]] > (.66 * tracks[i]->avgArea)))
			{
				// If track updated less than one time, than filter state is not correct.
				tracks[i]->KF->GetPrediction();

				if (assignment[i] != -1) // If we have assigned the detection, then update its coordinates,
				{
					tracks[i]->skipped_frames = 0;
					tracks[i]->prediction = tracks[i]->KF->Update(detections[assignment[i]], 1);
					tracks[i]->setArea(areaVec[assignment[i]]);
				}
				else				  // if not continue using predictions
				{
					tracks[i]->prediction = tracks[i]->KF->Update(Point2f(0, 0), 0);
				}

				if (tracks[i]->trace.size() > (size_t)max_trace_length)
				{
					tracks[i]->trace.erase(tracks[i]->trace.begin(), tracks[i]->trace.end() - max_trace_length);
				}

				tracks[i]->trace.push_back(tracks[i]->prediction);
				tracks[i]->KF->LastResult = tracks[i]->prediction;
			}
		}
	}
}

/*
//Update tracker from centroids found
void CTracker::Update(vector<Point2d>& detections, vector<double>& areaVec)
{
	// if no tracks exist, each detection (center) is a new track 
	if (tracks.size() == 0)
	{
		for (size_t i = 0; i<detections.size(); i++)
		{
			CTrack* tr = new CTrack(detections[i], dt, Accel_noise_mag);
			tr->setArea(areaVec[i]);
			tracks.push_back(tr);
		}
	}

	// -----------------------------------
	// Здесь треки уже есть в любом случае
	// -----------------------------------
	int T = tracks.size();		// треки
	int D = detections.size();	// детекты

	// Матрица расстояний от N-ного трека до M-ного детекта.
	vector< vector<double> > Cost(T, vector<double>(D));
	vector<int> assignment; // назначения

	// -----------------------------------
	// Треки уже есть, составим матрицу расстояний
	// -----------------------------------
	double dist;
	for (size_t i = 0; i<tracks.size(); i++)
	{
		//Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for (size_t j = 0; j<detections.size(); j++)
		{
			Point2d diff = (tracks[i]->prediction - detections[j]);
			dist = sqrt(diff.x*diff.x + diff.y*diff.y);
			Cost[i][j] = dist;
		}
	}
	// -----------------------------------
	// Solving assignment problem (tracks and predictions of Kalman filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

	// -----------------------------------
	// clean assignment from pairs with large distance
	// -----------------------------------
	// Not assigned tracks
	vector<int> not_assigned_tracks;

	for (size_t i = 0; i<assignment.size(); i++)
	{
		if (assignment[i] != -1)
		{
			if (Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i] = -1;
				// Mark unassigned tracks, and increment skipped frames counter,
				// when skipped frames counter will be larger than threshold, track will be deleted.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{
			// If track doesn't have an assigned detection, then increment skipped frames counter.
			tracks[i]->skipped_frames++;
		}

	}

	// If a track didn't get a new detection for a long time, remove the track.
	for (size_t i = 0; i<tracks.size(); i++)
	{
		if (tracks[i]->skipped_frames > (size_t) maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin() + i);
			assignment.erase(assignment.begin() + i);
			i--;
		}
	}
	
	// find detections that have not been assigned
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for (size_t i = 0; i<detections.size(); i++)
	{
		it = find(assignment.begin(), assignment.end(), i);
		if (it == assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	//then start new tracks for them
	if (not_assigned_detections.size() != 0)
	{
		for (size_t i = 0; i<not_assigned_detections.size(); i++)
		{
			CTrack* tr = new CTrack(detections[not_assigned_detections[i]], dt, Accel_noise_mag);
			tr->setArea(areaVec[not_assigned_detections[i]]);
			//areaVec.erase(areaVec.begin()+not_assigned_detections[i]);
			tracks.push_back(tr);
		}
	}

	// Update the state of the Kalman Filters 
	for (size_t i = 0; i<assignment.size(); i++)
	{
		// If track updated less than one time, than filter state is not correct.
		tracks[i]->KF->GetPrediction();

		if (assignment[i] != -1) // If we have assigned the detection, then update its coordinates,
		{
			tracks[i]->skipped_frames = 0;
			tracks[i]->prediction = tracks[i]->KF->Update(detections[assignment[i]], 1);
			tracks[i]->setArea(areaVec[assignment[i]]);
		}
		else				  // if not continue using predictions
		{
			tracks[i]->prediction = tracks[i]->KF->Update(Point2f(0, 0), 0);
		}

		if (tracks[i]->trace.size()>(size_t)max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(), tracks[i]->trace.end() - max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult = tracks[i]->prediction;
		//tracks[i]->setArea(areaVec[i]);
	}
	//create output file
	ofstream file;
	file.open("../objects/TrackerArea.txt", std::ofstream::out | std::ofstream::app);

	for (size_t i = 0; i < tracks.size(); i++)
	{

		file << "ID: " << tracks[i]->track_id << " A: " << tracks[i]->getArea() << endl;
	}

	file.close();

}
*/

//Tracker Destructor 
CTracker::~CTracker(void)
{
	for(size_t i=0;i<tracks.size();i++)
	{
		delete tracks[i];
	}
	tracks.clear();
}
