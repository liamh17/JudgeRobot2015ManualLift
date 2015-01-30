package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.control.*;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

/**
 * This class is intended to be a child class for a "PickupTote" task.  Possibly needs to not itself be a Task?
 * Parent class will be responsible for communicating with Raspberry Pi and determining if it's OK for the
 * driver to be in control, or if we should continue to take control here.
 *
 * @author K. Loux
 */
public class MotionTask extends TaskBase
{
	// Fields
	private RobotPostionEstimator posEstimator;// TODO:  Create this class -> this should interface to Raspberry Pi through UDP socket
	private HolonomicRobotDrive drive;
	private MotionTaskController posCmdGenerator;
	private RobotKinematicsController posController;
	private RobotKinematicsController velController;
	
	public MotionTask(HolonomicRobotDrive _drive, RobotPositionEstimator _posEstimator,
			RobotKinematics targetPosition)
	{
		super("Motion Task");
		drive = _drive;
		posEstimator = _posEstimator;
		
		posCmdGenerator = new MotionTaskController(posEstimator.GetCurrentPosition(),
				targetPosition, drive.GetMaxXVel(), maxAcc, RobotConfiguration.freq);// TODO:  Don't hardcode values here; use RobotConfig class
		posController = new RobotKinematicsController(1.0, 0.0, 200, RobotConfiguration.freq);// TODO:  Don't hardcode values here; use RobotConfig class
	}
	
	protected void ProcessState()
	{
		// position estimator should be updated in the class where it is owned (where it is created - here it is only
		// referenced - it's created in the part class)
		RobotKinematics velCmd = posController.DoControl(posCmdGenerator.Update(), posEstimator.GetCurrentPosition());
		
		// I think we can do without the velocity controller here - let's start out that way, anyway
		drive.Drive(velCmd.x, velCmd.y, velCmd.theta);
	}
	
	protected boolean IsComplete()
	{
		return posCmdGenerator.ReachedTarget(posEstimator.GetCurrentPosition(), 1.0);// TODO:  Don't hardcode this, and we'll need to tune it to see what is reasonable
	}
	
	protected void OnRemoveTask()
	{
		// Nothing here now
	}
	
	public String GetStateName()
	{
		return "Motion Task";// Only one state for this task
	}
	
	protected boolean OkToDrive()
	{
		return false;// Robot is always in control for this task
	}
}
