package org.usfirst.frc.team3167.vision;

import org.usfirst.frc.team3167.drive.RobotKinematics;

/**
 * Abstract base class for a vision tracking class.
 */
public abstract class Tracker 
{
	protected abstract void init();
	
	protected abstract boolean seesTote();
	
	protected abstract void doAnalysis();
	
	protected abstract RobotKinematics getPosition();
}
