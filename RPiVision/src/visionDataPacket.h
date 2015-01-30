// File:  visionDataPacket.h
// Date:  1/30/2015
// Auth:  K. Loux
// Desc:  Datagram packet for Raspberry Pi-to-RoboRIO communication.  This MUST
//        exactly match the packet defined in the RoboRIO code.

#ifndef VISION_DATA_PACKET_H_
#define VISION_DATA_PACKET_H_

struct RobotKinematicData
{
	double x;// [in] or [in/sec]
	double y;// [in] or [in/sec]
	double theta;// [rad] or [rad/sec]
};

enum TargetLockType
{
	TargetLockNarrow,
	TargetLockWide,
	TargetLockNone
};

struct VisionDataPacket
{
	// Position and velocity with respect to the target
	// Target position is always (0,0,0)
	// Coordinate system is:
	//    x -> TODO:  Make choose this so it is convenient to use with robot coordinate system
	//    y ->TODO:  Make choose this so it is convenient to use with robot coordinate system
	//    theta -> positive is CCW rotation when viewed from above (ensure above csys obeys RH rule)
	RobotKinematicData positionWRTTarget; 
	RobotKinematicData velocityWRTTarget;
	TargetLockType targetLock;
};

#endif// VISION_DATA_PACKET_H_