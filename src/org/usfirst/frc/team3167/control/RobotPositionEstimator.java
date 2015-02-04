package org.usfirst.frc.team3167.control;

import org.usfirst.frc.team3167.drive.RobotKinematics;

public class RobotPositionEstimator
{
	
	private RobotKinematics position;
	private boolean seesTote;
	
	public RobotPositionEstimator()
	{
		position.x = 0;
		position.y = 0;
		position.theta = 0;
	}
	
	public synchronized void setSeesTote(boolean seesTote)
	{
		this.seesTote = seesTote;
	}
	
	public synchronized void setPosition(RobotKinematics position)
	{
		this.position = position;
	}
	
	public synchronized RobotKinematics GetCurrentPosition()
	{
		return position;
	}
	
	public synchronized boolean SeesTarget()
	{
		return seesTote;
	}
}
