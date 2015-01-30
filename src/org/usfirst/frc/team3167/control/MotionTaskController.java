package org.usfirst.frc.team3167.control;

import org.usfirst.frc.team3167.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.drive.RobotKinematics;

/**
 * Object for generating time-varying position commands to feed into robot position controller.
 * This is the first object in the controller chain -> tell it where you are and where you
 * want to go, and it feeds the position controller with commands that provide a smooth transition
 * between initial and final positions.
 *
 * @author K. Loux
 */
public class MotionTaskController
{
	// Fields
	private RobotKinematics endPosition;
	private RobotKinematics currentCmdPosition;
	private RobotKinematics vel;// [per second]
	
	private SecondOrderLimiter xLimiter;
	private SecondOrderLimiter yLimiter;
	private SecondOrderLimiter thetaLimiter;

    // Methods
	/**
	 * Constructor for limiting signals symmetrically around zero.
	 *
	 * @param currentPosition	Current position of the robot
	 * @param _endPosition		Target end position
	 * @param maxVel 			Limit on the magnitude of the signal's first derivative
	 * @param maxAccel			Limit on the magnitude of the signal's second derivative
	 * @param freq 			Fixed frequency at which the object will be updated [Hz]
	 */
    public MotionTaskController(RobotKinematics currentPosition,
    		RobotKinematics _endPosition, RobotKinematics maxVel,
    		RobotKinematics maxAccel, double freq, boolean smoothMotion)
    {
    	endPosition = _endPosition;
    	currentCmdPosition = currentPosition;
    	vel = maxVel;
    	
    	// Optionally, we could pass the maxVel and maxAccel arguments directly to the limiter,
    	// but let's be a little fancier and attempt to get the robot to arrive at all three
    	// targets at exactly the same time.  We provide this option, because we might actually
    	// prefer one over the other (i.e. smooth move to position1, then normal move to tote
    	// pick-up position, so we ensure that the robot is as square as possible during the move).
    	RobotKinematics modifiedVel = new RobotKinematics();
    	RobotKinematics modifiedAccel = new RobotKinematics();
    	if (smoothMotion)
    	{
    		// Figure out which move will take the longest time, and use that time as the target time
	    	double xTime = ComputeTimeForMove(endPosition.x - currentPosition.x,
	    			maxVel.x, maxAccel.x);
	    	double yTime = ComputeTimeForMove(endPosition.y - currentPosition.y,
	    			maxVel.y, maxAccel.y);
	    	double thetaTime = ComputeTimeForMove(endPosition.theta - currentPosition.theta,
	    			maxVel.theta, maxAccel.theta);
	    	
	    	// Now scale the allowed velocities and acceleration for the other moves so
	    	// they take the same amount of time as the longest move
	    	double scale;
	    	if (xTime > yTime && xTime > thetaTime)// x move is the longest
	    	{
	    		modifiedVel.x = maxVel.x;
	    		modifiedAccel.x = maxAccel.x;
	    		
	    		scale = ComputeMoveScale(endPosition.y - currentPosition.y,
	    				maxVel.y, maxAccel.y, xTime);
	    		modifiedVel.y = maxVel.y * scale;
	    		modifiedAccel.y = maxAccel.y * scale;
	    		
	    		scale = ComputeMoveScale(endPosition.theta - currentPosition.theta,
	    				maxVel.theta, maxAccel.theta, xTime);
	    		modifiedVel.theta = maxVel.theta * scale;
	    		modifiedAccel.theta = maxAccel.theta * scale;
	    	}
	    	else if (yTime > thetaTime)// y move is the longest
	    	{
	    		modifiedVel.y = maxVel.y;
	    		modifiedAccel.y = maxAccel.y;
	    		
	    		scale = ComputeMoveScale(endPosition.x - currentPosition.x,
	    				maxVel.x, maxAccel.x, yTime);
	    		modifiedVel.x = maxVel.x * scale;
	    		modifiedAccel.x = maxAccel.x * scale;
	    		
	    		scale = ComputeMoveScale(endPosition.theta - currentPosition.theta,
	    				maxVel.theta, maxAccel.theta, yTime);
	    		modifiedVel.theta = maxVel.theta * scale;
	    		modifiedAccel.theta = maxAccel.theta * scale;
	    	}
	    	else// theta move is the longest
	    	{
	    		modifiedVel.theta = maxVel.theta;
	    		modifiedAccel.theta = maxAccel.theta;
	    		
	    		scale = ComputeMoveScale(endPosition.x - currentPosition.x,
	    				maxVel.x, maxAccel.x, thetaTime);
	    		modifiedVel.x = maxVel.x * scale;
	    		modifiedAccel.x = maxAccel.x * scale;
	    		
	    		scale = ComputeMoveScale(endPosition.y - currentPosition.y,
	    				maxVel.y, maxAccel.y, thetaTime);
	    		modifiedVel.y = maxVel.y * scale;
	    		modifiedAccel.y = maxAccel.y * scale;
	    	}
    	}
    	else
    	{
    		modifiedVel = maxVel;
    		modifiedAccel = maxAccel;
    	}
    	
    	xLimiter = new SecondOrderLimiter(modifiedVel.x, modifiedAccel.x, freq);
    	yLimiter = new SecondOrderLimiter(modifiedVel.y, modifiedAccel.y, freq);
    	thetaLimiter = new SecondOrderLimiter(modifiedVel.theta, modifiedAccel.theta, freq);
    }
    
    private double ComputeTimeForMove(double x, double v, double a)
    {
    	double accelTime = v / a;
    	double accelDist = 0.5 * a * accelTime * accelTime;
    	double constVelDist = x - 2 * accelDist;
    	double constVelTime;
    	if (constVelDist < 0)// We don't have far enough to go to reach our max velocity - we accelerate the entire time
    	{
    		constVelTime = 0;
    		accelTime = Math.sqrt(2 * x / a);
    	}
    	else
    		constVelTime = constVelDist / v;
    	
    	return 2 * accelTime + constVelTime;
    }
    
    private double ComputeMoveScale(double x, double v, double a, double t)
    {
    	if (x - a * t * t > 0)// Normal case
    		return x / (v * t - v * v / a);
    	else// No constant velocity segment
    		return 4 * x / (a * t * t);
    }
    
    /**
	 * Method to be called exactly once per cycle while this object is active.
	 *
	 */
    public RobotKinematics Update()
    {
    	currentCmdPosition.x = xLimiter.Process(endPosition.x);
    	currentCmdPosition.y = yLimiter.Process(endPosition.y);
    	currentCmdPosition.theta = thetaLimiter.Process(endPosition.theta);
    	return currentCmdPosition;
    }
    
    /**
	 * Returns the position command calculated during the last call to Update()
	 *
	 */
    public RobotKinematics GetCurrentPositionCmd()
    {
    	return currentCmdPosition;
    }
    
    public boolean ReachedTarget(RobotKinematics currentPosition, double errorTolerance)
    {
    	// TODO:  We may find that certain directions matter more than others, and we
    	// can apply scaling factors here to accommodate for that.  For now, assume
    	// each direction is just as important as others.
    	double xError = currentPosition.x - endPosition.x;
    	double yError = currentPosition.y - endPosition.y;
    	double thetaError = (currentPosition.theta - endPosition.theta) * vel.x / vel.theta;
    	double errorSq = xError * xError + yError * yError + thetaError * thetaError;
    	
    	return errorSq < errorTolerance * errorTolerance;
    }
}
