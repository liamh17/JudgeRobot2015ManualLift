// File:  targetTracker.h
// Date:  9/3/2013
// Auth:  K. Loux
// Desc:  Target tracker implementing SIFT a la openCV.

#ifndef TARGET_TRACKER_H_
#define TARGET_TRACKER_H_

// Local headers
#include "visionDataPacket.h"

class TargetTracker
{
public:
	TargetTracker();
	
	// Mark - this is the place to put all of your stuff converted from Python
	// Any helper methods you have should go in the "private" section of this class,
	// or in another helper class(es)
	// I think that training, updating the tracker, and getting data out are the only
	// public interfaces we need for the class.  Is this right?
	bool Train(const std::string &imageFileName);
	void DoTracking();
	
	inline bool SeesTarget() const { return seesTarget; }
	
	inline RobotKinematicData GetPosition() const { return position; }
	inline RobotKinematicData GetPositionDelta() const { return positionDelta; }

private:
	RobotKinematicData position;// robot position w.r.t. target
	RobotKinematicData positionDelta;
	
	bool seesTarget;
	
	// TODO:  Populate with helper methods/objects
	// It's probably best to have a helper class for calculating the position/orientation from
	// the SIFT output.  You would then add that class here.  This will make testing easier,
	// and keep everything more modular.
};

#endif// TARGET_TRACKER_H_