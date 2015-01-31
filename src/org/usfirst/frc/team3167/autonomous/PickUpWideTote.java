package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.control.RobotPositionEstimator;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.RobotKinematics;

public class PickUpWideTote extends PickUpToteTask
{
	private final static RobotKinematics lineUpPosition = new RobotKinematics(0, 10, 0);// TODO:  Put good numbers in here
	private final static RobotKinematics liftPosition = new RobotKinematics(0, 4, 0);// TODO:  Put good numbers in here
	
	public PickUpWideTote(HolonomicRobotDrive drive, Lift lift,
			RobotPositionEstimator posEstimator)
	{
		super(drive, lift, posEstimator, "Wide", lineUpPosition, liftPosition);
	}
}
