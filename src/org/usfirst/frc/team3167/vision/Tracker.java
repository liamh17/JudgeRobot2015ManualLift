package org.usfirst.frc.team3167.vision;

import org.usfirst.frc.team3167.drive.RobotKinematics;

public abstract class Tracker 
{
	protected abstract void init();
	
	protected abstract boolean seesTote();
	
	protected abstract void doAnalysis();
	
	protected abstract RobotKinematics getPosition();
}
