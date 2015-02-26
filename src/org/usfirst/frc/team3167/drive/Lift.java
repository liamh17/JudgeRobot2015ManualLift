package org.usfirst.frc.team3167.drive;

import edu.wpi.first.wpilibj.CANJaguar;

import org.usfirst.frc.team3167.robot.RobotConfiguration;
import org.usfirst.frc.team3167.util.DigitalSwitch;

public class Lift extends BangBangAxis
{
	private static final double homingSpeed = 1.0;// [in/sec]
	private static final double homingAccel = 0.5;// [in/sec^2]
	private static final double normalSpeed = 4.0;// [in/sec]
	private static final double normalAccel = 4.0;// [in/sec^2]
		
	private static final double bottom = 0;// [in]
	private static final double aboveGroundTote = 0;// [in]
	
	public Lift(int canID, DigitalSwitch homeSwitch, double homeSwitchPosition, double sprocketPitchCircumference, boolean reverse, int encPPR, double tolerance)
	{
		super(new CANJaguar(canID), normalSpeed, normalAccel, homingSpeed, homingAccel, homeSwitch,
				sprocketPitchCircumference, homeSwitchPosition, tolerance, reverse, encPPR);
	}
	
	public void GoToPosition(double endPosition)
	{
		SetCmdPosition(endPosition);
	}
	
	public boolean IsLifted()
	{
		// TODO: Return whether or not the tote is off of the ground (i.e. OK to drive)
		return true;
	}
	
	public double GetHeight()
	{
		return GetPosition();
	}
}
