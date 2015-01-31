package org.usfirst.frc.team3167.drive;

import org.usfirst.frc.team3167.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

public class Lift
{
	// TODO:  Implement
	// We'll do position control (and speed control?)
	// Need to have limit switch(es) so we know starting position
	
	private static final double bottom = 0;// [in]
	private static final double aboveGroundTote = 0;// [in]
	
	private boolean isHomed;
	private static final double homingSpeed = 0.5;// [in/sec]
	private static final double homingAccel = 0.5;// [in/sec]
	
	private static final double normalSpeed = 2;// [in/sec]
	private static final double normalAccel = 2;// [in/sec]
	
	private SecondOrderLimiter cmdGenerator;
	
	private double cmdPosition;
	
	public Lift()
	{
		// TODO:  Create the limit switch and encoder objects (or use CAN for encoder feedback?)
		isHomed = false;
		cmdGenerator = new SecondOrderLimiter(homingSpeed, homingAccel,
				RobotConfiguration.frequency);
		GoToPosition(-9999);// Something very far below our current position
	}
	
	public void GoToPosition(double endPosition)
	{
		endPosition = cmdPosition;
	}
	
	public void Update()
	{
		// TODO:  Implement
		/*if (!isHomed && switch is depressed)
		{
			isHomed = true;
			Reset encoder to bottom position
			cmdGenerator = new SecondOrderLimiter(normalSpeed, normalAccel,
				RobotConfiguration.frequency);
			GoToPosition(bottom);
		}*/
		
		cmdGenerator.Process(cmdPosition);// TODO:  Use this value to close the loops
		
		// TODO:  Close loops here
	}
	
	public boolean IsLifted()
	{
		// TODO: Return whether or not the tote is off of the ground (i.e. OK to drive)
		return true;
	}
	
	public double GetHeight()
	{
		return 0;// TODO:  Need to figure out what we want this to report -> what is the reference?  top of arm? bottom of tote?
	}
}
