
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.TaskManager;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.Wheel;
import org.usfirst.frc.team3167.util.Conversions;
import org.usfirst.frc.team3167.util.DigitalSwitch;
import org.usfirst.frc.team3167.util.JoystickButton;
import org.usfirst.frc.team3167.util.SecondOrderFilter;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	private HolonomicRobotDrive drive;
	private Lift narrowLift = new Lift(RobotConfiguration.narrowToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.narrowToteHomeChannel, true),
			RobotConfiguration.narrowHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference, false);
	private Lift wideLift = new Lift(RobotConfiguration.wideToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.wideToteHomeChannel, true),
			RobotConfiguration.wideHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference, true);
	
	// TODO:  Add cameras and trackers
	private TaskManager taskManager = new TaskManager();
	private Joystick driveJoystick1 = new Joystick(1);
	private Joystick driveJoystick0 = new Joystick(0);
	
	//private RobotDrive drive;
	
	Preferences prefs = Preferences.getInstance();
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	//filter = new SecondOrderFilter(prefs.getDouble("cutoff", 3.0), prefs.getDouble("damp", 0.0), RobotConfiguration.frequency);
    	//testSig = prefs.getDouble("testSig", 0.0);
    	//prefs.putDouble("Kp", 1.0);
    	//prefs.putDouble("Ki", 0.0);
    	BuildRobotDrive();
    	// TODO:  Create all the stuff herej
    }
    
    private void DoCommonUpdates()
    {
    	updateSmartDashboard();
    	taskManager.DoCurrentTask();
    	if (taskManager.OkToDrive())
    	{
    		try
	    	{
	    		drive.Drive(driveJoystick1);
	    	}
	    	catch (Exception ex)
	    	{
	    		System.out.println("Failed to drive: " + ex.getMessage());
	    	}
    		
    	}
    	
    	narrowLift.Update();
    	wideLift.Update();
    	
    	// TODO:  Update all of our common objects
    	// Both target trackers
    	// Anything else?
    	
    	// CAN objects - this line sends the commands
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    }
    
    public void updateSmartDashboard()
    {
    	//SmartDashboard.putNumber("Front Right Wheel Angle", drive.GetWheel(1).GetWheelAngle());
    	//SmartDashboard.putNumber("Rear Left Wheel Angle", drive.GetWheel(2).GetWheelAngle());
    	//SmartDashboard.putNumber("Rear Right Wheel Angle", drive.GetWheel(3).GetWheelAngle());
    	
    	//SmartDashboard.putNumber("Left Front Wheel KP", drive.GetWheel(0).getKP());
    	//SmartDashboard.putNumber("Left Front Wheel KI", drive.GetWheel(0).getKI()); 
    	//SmartDashboard.putNumber("Left Front Wheel KD", drive.GetWheel(0).getKD()); 
    	//SmartDashboard.putNumber("testSig", testSig); 
    	
    	//SmartDashboard.putNumber("Left Front Wheel speed", drive.GetWheel(0).GetWheelVelocity());
    	//SmartDashboard.putNumber("Left Front Wheel command speed", drive.GetWheel(0).getCANMotor().get() / drive.GetWheel(0).getGearRatio() * Conversions.RPMToRadPerSec);
    }
    
    public void autonomousInit()
    {
    	System.out.println("autonomousInit");
    	
    	taskManager.ClearAllTasks();
    	
    	if (true)
    	{
    		//taskManager.AddTask(new DriveForwardTask());// TODO:  What do we want to do for autonomous?
    	}
    	// TODO Add more options + ways to select
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	DoCommonUpdates();
    	// TODO:  Do stuff here
    }
    
    public void teleopInit()
    {
    	System.out.println("teleopInit");
    	
    	taskManager.ClearAllTasks();
    }
    
    /**
     * Function called as fast as possible during operator control.
     * Handles button presses only.
     */
    public void teleopContinuous()
    {
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	DoCommonUpdates();
    	System.out.println(wideLift.GetPosition());
    	
    	if(driveJoystick1.getRawButton(3))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideHomeSwitchHeight);
    	}
    	else if(driveJoystick1.getRawButton(5))
    	{
    		wideLift.GoToPosition(RobotConfiguration.widePickupToteHeight); // Pick up a tote by rasing 4 in
    	}
    	else if(driveJoystick1.getRawButton(4))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideRaiseToteToStack); // Pick up a tote by rasing 4 in
    	}
    	else if(driveJoystick1.getRawButton(6))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideStackingToteOnTote);
    	}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
    }
    
    public void disabledInit()
    {
    	// Call test print statements to execute only one time
    	//drive.PrintTest();
    }
    
    public void disabledPeriodic()
    {
    	//updateSmartDashboard();
    	//drive.PrintTest();
    	// Call test print statements to execute every cycle
    	//System.out.println(GenerateLiftTestString("Narrow", narrowLift));
    	//System.out.println(GenerateLiftTestString("Wide", wideLift));
    	//System.out.println(drive.GetWheelAngleString());
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    	System.out.println("X: " + driveJoystick1.getX() + ", Y: " + driveJoystick1.getY() + ", twist: " + driveJoystick1.getTwist());

    }
    
    private String GenerateLiftTestString(String name, Lift lift)
    {
    	String s = name;
    	s += " Home Switch = " + lift.getSwitch().IsPressed() + "; ";
    	s += " Position = " + lift.GetPosition() + " in; ";
    	s += " Encoder = " + lift.GetEncoderAngle() + " deg";
    	return s;
    }
    
    private void BuildRobotDrive()
    {
    	drive = new HolonomicRobotDrive(RobotConfiguration.frequency);
    	
    	double maxSpeedScale = 0.8;// Limit the speed because we know we can't actually reach motor free-running speed
    	double maxWheelSpeed = RobotConfiguration.wheelMotorMaxSpeed
    			/ RobotConfiguration.wheelGearboxRatio
    			* Conversions.RPMToRadPerSec * maxSpeedScale;// [rad/sec]
    	double halfTrack = RobotConfiguration.track / 2;// [in]
    	double halfWheelbase = RobotConfiguration.wheelbase / 2;// [in]
    	double leftWheelAxisX = -1;
    	double rightWheelAxisX = -leftWheelAxisX;
    	double rollerAngle1 = RobotConfiguration.rollerAngle;
    	double rollerAngle2 = -rollerAngle1;
    	//double gearRatio = 1;// 1 because the encoder rotates with the wheel, not the motor
    	
    	// Left front
    	drive.AddWheel(new Wheel(-halfTrack, halfWheelbase, leftWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, 
    			maxWheelSpeed, RobotConfiguration.leftFrontMotorID));
    	
    	
    	// Right front
    	drive.AddWheel(new Wheel(halfTrack, halfWheelbase, rightWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, maxWheelSpeed, RobotConfiguration.rightFrontMotorID));
    	
    	// Left rear
    	drive.AddWheel(new Wheel(-halfTrack, -halfWheelbase, leftWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius,
    			maxWheelSpeed, RobotConfiguration.leftRearMotorID));
    	
    	// Right rear
    	drive.AddWheel(new Wheel(halfTrack, -halfWheelbase, rightWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, maxWheelSpeed, RobotConfiguration.rightRearMotorID));
    	
    	
    	// Add acceleration limits
    	drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

    	// Set the deadband for the joysticks
    	drive.SetDeadband(0.1);// horizontal deadband
    	drive.SetMinimumOutput(0.1);// vertical deadband
    	
    	drive.Initialize();
    }
    
}
