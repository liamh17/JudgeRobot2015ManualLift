package org.usfirst.frc.team3167.drive;

public class RobotKinematics
{
	public double x;// [in]
	public double y;// [in]
	public double theta; //[rad]
	
	public RobotKinematics()
	{
		x = 0;
		y = 0;
		theta = 0;
	}
	
	public RobotKinematics(double x, double y, double theta)
	{
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
}
