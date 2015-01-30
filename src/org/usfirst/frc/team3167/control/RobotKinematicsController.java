package org.usfirst.frc.team3167.control;

import org.usfirst.frc.team3167.util.PIDControllerII;
import org.usfirst.frc.team3167.drive.RobotKinematics;

/**
 * Object for implementing 3-element control for robot kinematics.  Can be used as position or velocity controller.
 *
 * @author K. Loux
 */
public class RobotKinematicsController
{
	private PIDControllerII xController;
	private PIDControllerII yController;
	private PIDControllerII thetaController;
	
	// For right now, assume the same gains work for all controllers
	// TODO:  Scale thetaController gains by maxLinearVel / maxRotationVel
	public RobotKinematicsController(double Kp, double Ki, int queueSize, double freq)
	{
		xController = new PIDControllerII(Kp, Ki, queueSize, freq);
		yController = new PIDControllerII(Kp, Ki, queueSize, freq);
		thetaController = new PIDControllerII(Kp, Ki, queueSize, freq);
	}
	
	public RobotKinematics DoControl(RobotKinematics cmd, RobotKinematics actual)
	{
		RobotKinematics output = new RobotKinematics();
		output.x = xController.DoControl(cmd.x, actual.x);
		output.y = yController.DoControl(cmd.y, actual.y);
		output.theta = thetaController.DoControl(cmd.theta, actual.theta);
		return output;
	}
}
