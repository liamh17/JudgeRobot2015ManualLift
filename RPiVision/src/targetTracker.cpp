// File:  targetTracker.cpp
// Date:  9/3/2013
// Auth:  K. Loux
// Desc:  Target tracker implementing SIFT a la openCV.

// Local headers
#include "targetTracker.h"

// Constructor
TargetTracker::TargetTracker()
{
}

bool TargetTracker::Train(const std::string &imageFileName)
{
	// TODO:  OpenCV stuff here
	// One reason to return false is if the specified file doesn't actually exist...
	// Are there any other reasons?  Can the algorithm itself fail somehow?
}

void TargetTracker::DoTracking()
{
	// TODO:  OpenCV stuff here
	// if it's possible that this method fails, make it return a bool instead of void
	// The goal of this method is to update:
	//    - seesTarget (every frame)
	//    - position (only if we see the target)
	//    - positionDelta (only if we see the target)
}