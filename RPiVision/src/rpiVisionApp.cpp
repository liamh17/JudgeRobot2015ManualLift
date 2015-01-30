// File:  rpiVisionApp.cpp
// Date:  1/30/2015
// Auth:  K. Loux
// Desc:  Application entry point for vision processing application.

// Standard C++ headers
#include <cstdlib>
#include <iostream>
#include <string>

// Local headers
#include "targetTracker.h"
#include "visionDataPacket.h"
#include "cppSocket.h"
#include "timingUtility.h"

// Globals
const unsigned short localPort(3167);
const unsigned short remotePort(5000);// TODO:  Look this up in the rules, pick a port, and make sure it matches configuration in RobotRIO code
const std::string roboRioIP("192.168.31.67");// TODO:  Set this to the correct value

const int returnOK(0);
const int returnError(1);

// We know we need to communicate at 60 Hz with the RoboRIO, but if we can do vision
// processing faster than that, we should (it will give us a better velocity estimate
// - this is also the reason we have separate position and velocity fields in the
// data packet).  Better to design for this felxibility from the start and not use it
// (i.e. set the timers to be the same) than to decide we want it later and have to
// re-design.  We will do this in a very simplistic way, and assume that the vision
// loop time is an integer quotient of the communication time.
const double communicationTime(1.0 / 60.0); // [sec]
const double visionProcessTime(communicationTime / 1.0); // [sec] TODO:  Determine if we can set the divisor to something greater than 1.0 (must be an integer!)

// Application entry point
int main(int, char*[])
{
	CPPSocket socket(CPPSocket::SocketUDPClient);
	VisionDataPacket packet;
	
	TimingUtility loopTimer(visionProcessTime);
	const unsigned int commModulo(communicationTime / visionProcessTime);
	
	// Mark - not sure how your code is structured, but I'm thinking that it
	// might be wise to build something modeled like this, so we can initialize
	// with different training images, then have all the rest of the code be
	// the same.  Code re-use is king!  Modify to suit if I'm off-base.
	TargetTracker narrowToteTracker;
	TargetTracker wideToteTracker;
	RobotKinematicData narrowTotePosition;
	RobotKinematicData wideTotePosition;
	
	// ========================================================================
	// Initialization
	// ========================================================================
	std::cout << "Ready to initialize" << std::endl;
	if (!socket.Create(localPort))
	{
		std::cerr << "Failed to create socket" << std::endl;
		return returnError;
	}
	else if (!socket.SetBlocking(false))
	{
		std::cerr << "Failed to set non-blocking mode" << std::endl;
		return returnError;
	}
	
	// Initialize the vision trackers, too
	// Not sure what this will need to look like?  Do we pass a camera object here?
	// Is the camera object a separate object, or contained within these classes?
	// If we have only one camera, the loop might need to be something like:
	//   image = GetFromCamera();
	//   narrowToteTracker.DoTracking(image);
	//   wideToteTracker.DoTracking(image);
	// instead of what I have now, in order to be more efficient
	if (!narrowToteTracker.Train("logoImage.bmp"))// TODO:  Use proper files here
	{
		std::cerr << "Failed to train narrow tote tracker" << std::endl;
		return returnError;
	}
	else if (!wideToteTracker.Train("wideToteImage.bmp"))// TODO:  Use proper files here
	{
		std::cerr << "Failed to train wide tote tracker" << std::endl;
		return returnError;
	}
	
	// ========================================================================
	// Main loop
	// ========================================================================
	std::cout << "Entering main loop" << std::endl;
	unsigned int loopCount(0);
	while (true)// never stop
	{
		loopTimer.TimeLoop();// Ensures we execute at the proper frequency
		
		narrowToteTracker.DoTracking();
		wideToteTracker.DoTracking();
		
		if (++loopCount % commModulo == 0)// Check to see if we should send data
		{
			if (narrowToteTracker.SeesTarget())
			{
				packet.targetLock = TargetLockNarrow;
				packet.positionWRTTarget = narrowToteTracker.GetPosition();
				packet.velocityWRTTarget =
					narrowToteTracker.GetPositionDelta() / visionProcessTime;
			}
			else if (wideToteTracker.SeesTarget())
			{
				packet.targetLock = TargetLockWide;
				packet.positionWRTTarget = wideToteTracker.GetPosition();
				packet.velocityWRTTarget =
					wideToteTracker.GetPositionDelta() / visionProcessTime;
			}
			else
				packet.targetLock = TargetLockNone;
			
			// Even if the send fails, don't quit
			if (!socket.UDPSend(roboRioIP.c_str(), remotePort, (DataType*)&packet, sizeof(packet)))
				std::cerr << "Failed to send packet:  " << socket.GetErrorString() << std::endl;
		}
	}
	
	return returnOK;
}