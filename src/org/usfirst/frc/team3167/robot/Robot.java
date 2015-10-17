
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.DriveAutoTask;

import org.usfirst.frc.team3167.autonomous.PickupTrashcanTask;
import org.usfirst.frc.team3167.autonomous.TaskManager;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.Wheel;
import org.usfirst.frc.team3167.util.Conversions;
import org.usfirst.frc.team3167.util.DigitalSwitch;
import org.usfirst.frc.team3167.util.JoystickButton;
import org.usfirst.frc.team3167.util.SecondOrderFilter;
import org.usfirst.frc.team3167.vision.LogoTracker;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

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
	/*private Lift narrowLift = new Lift(RobotConfiguration.narrowToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.narrowToteHomeChannel, true),
			RobotConfiguration.narrowHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference, true, 
			RobotConfiguration.narrowEncoderPPR, RobotConfiguration.narrowLiftTolerance);*/
	private CANJaguar narrowLift = new CANJaguar(RobotConfiguration.narrowToteLiftMotorID);
	//private Lift wideLift = new Lift(RobotConfiguration.wideToteLiftMotorID,
			//new DigitalSwitch(RobotConfiguration.wideToteHomeChannel, true),
			//RobotConfiguration.wideHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference, true,
			//RobotConfiguration.wheelEncoderPPR, RobotConfiguration.wideLiftTolerance);
	private CANJaguar wideLift = new CANJaguar(RobotConfiguration.wideToteLiftMotorID);
	
	// TODO:  Add cameras and trackers
	private TaskManager taskManager = new TaskManager();
	private Joystick driveJoystick1 = new Joystick(1);
	//private Joystick driveJoystick0 = new Joystick(0);
	//private DigitalSwitch homeSwitch = new DigitalSwitch(7, false);
	
	//private RobotDrive drive;
	private boolean slideLock = false;
	
	private double speed = 0.0;
	  
	Preferences prefs = Preferences.getInstance();
	
	//enum Driver
	//{
	//	WIDE_DRIVER,
	//	NARROW_DRIVER;
	//}
	
	//private Driver driver = Driver.WIDE_DRIVER;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	//prefs.putDouble("Joystick min", 0.1);
    	SmartDashboard.putString("hello", "hello");
    	//filter = new SecondOrderFilter(prefs.getDouble("cutoff", 3.0), prefs.getDouble("damp", 0.0), RobotConfiguration.frequency);
    	//testSig = prefs.getDouble("testSig", 0.0);
    //	prefs.putDouble("Kp", 0.0);
    //	prefs.putDouble("Ki", 0.0);
    	BuildRobotDrive();
    	// TODO:  Create all the stuff here
    }
    
    private void DoCommonUpdates()
    {
    	try
    	{
	    	updateSmartDashboard();
	    	taskManager.DoCurrentTask();
	    	
	    	//drive.Drive(0.0, 30.0, 0.0);
	    	if (taskManager.OkToDrive())
	    	{
	    		//if(driver == Driver.WIDE_DRIVER)
	    		//{
		    		try 
			    	{
			    		drive.Drive(driveJoystick1, true, slideLock);
			    	}
			    	catch (Exception ex)
			    	{
			    		System.out.println("Failed to drive: " + ex.getMessage());
			    	}
	    		/*}
	    		else if(driver == Driver.NARROW_DRIVER)
	    		{
	    			try
			    	{
			    		drive.Drive(driveJoystick0, false, slideLock);
			    	}
			    	catch (Exception ex)
			    	{
			    		System.out.println("Failed to drive: " + ex.getMessage());
			    	}
	    		}*/
	    	}
	    	
	    	if(!isAutonomous())
	    	{	
	    		narrowLift.set(speed);
	    	}
	    	//narrowLift.Update();
	    	//wideLift.Update();
	    	
	    	//System.out.println(narrowLift.GetHeight());
	    	//System.out.println(wideLift.GetHeight());
	    	
	    	// TODO:  Update all of our common objects
	    	// Both target trackers
	    	// Anything else?
    	}
    	catch (Exception ex)
    	{
    		System.out.println(ex.toString());
    		ex.printStackTrace();
    	}
    	
    	// CAN objects - this line sends the commands
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    }
    
    public void updateSmartDashboard()
    {
    	
    	SmartDashboard.putNumber("Front Left Wheel Vel", drive.GetWheel(0).GetWheelVelocity());
    	SmartDashboard.putNumber("Front Right Wheel Vel", drive.GetWheel(1).GetWheelVelocity());
    	SmartDashboard.putNumber("Rear Left Wheel Vel", drive.GetWheel(2).GetWheelVelocity());
    	SmartDashboard.putNumber("Rear Right Wheel Vel", drive.GetWheel(3).GetWheelVelocity());
    	
    	SmartDashboard.putNumber("Front Left Cmd", (drive.GetWheel(0).getCANMotor().get()/drive.GetWheel(0).getGearRatio()) * Conversions.RPMToRadPerSec);
    	SmartDashboard.putNumber("Front Right Cmd", (drive.GetWheel(1).getCANMotor().get()/drive.GetWheel(1).getGearRatio()) * Conversions.RPMToRadPerSec);
    	SmartDashboard.putNumber("Rear Left Cmd", (drive.GetWheel(2).getCANMotor().get()/drive.GetWheel(2).getGearRatio()) * Conversions.RPMToRadPerSec);
    	SmartDashboard.putNumber("Rear Right Cmd", (drive.GetWheel(3).getCANMotor().get()/drive.GetWheel(3).getGearRatio()) * Conversions.RPMToRadPerSec);

    	
    	//SmartDashboard.putNumber("Front Left Wheel Angle", drive.GetWheel(0).GetWheelAngle());
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
    		//taskManager.AddTask(new PickupTrashcanTask(drive, 3.0, narrowLift, 3.5));
    		//taskManager.AddTask(new DriveAutoTask(drive, 3.0, wideLift, RobotConfiguration.widePickupToteHeight));
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
    	
    	/*if(driveJoystick1.getRawButton(1))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideHomeSwitchHeight);
    	}
    	else if(driveJoystick1.getRawButton(11))
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
    	else if(driveJoystick1.getRawButton(1))
    	{
    		//driver = Driver.WIDE_DRIVER;
    	}
    	else if(driveJoystick1.getRawButton(11))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideStackingOnToteAndStep);
    	}
    	else if(driveJoystick1.getRawButton(9))
    	{
    		wideLift.GoToPosition(RobotConfiguration.wideLoweringToteOnToteOnStep);
    	}
    	
    	if(driveJoystick1.getRawButton(3))
    	{
    		speed = -1.0;
    	}
    	else if(driveJoystick1.getRawButton(5))
    	{
    		speed = 1.0;
    	}
    	else
    	{
    		speed = 0.0;
    	} */
    	
        if(driveJoystick1.getRawButton(RobotConfiguration.narrowLiftUp))
        {
        	narrowLift.set(1.0);
        }
        else if(driveJoystick1.getRawButton(RobotConfiguration.narrowLiftDown))
        {
        	narrowLift.set(-1.0);
        }
        else
        {
        	narrowLift.set(0.0);
        }
        
        if(driveJoystick1.getRawButton(RobotConfiguration.wideLiftUp))
        {
        	wideLift.set(1.0);
        }
        else if(driveJoystick1.getRawButton(RobotConfiguration.wideLiftDown))
        {
        	double down = -1.0;
        	DigitalInput liam = new DigitalInput(1);

        	if(liam = 1)
        	{
        		down = 0.0; 
        	}
        	wideLift.set(down);
        }
        else
        {
        	wideLift.set(0.0);
        }
    	
    	// Enable a slide lock for sliding without turning
    	if(driveJoystick1.getRawButton(2) /*&& driver == Driver.WIDE_DRIVER*/)
    	{
    		slideLock = true;
    	}
    	else
    	{
    		slideLock = false;
    	}
    	
    	
    	//else if(driver == Driver.WIDE_DRIVER)
    	//{
    	//	slideLock = false;
    	//}
    	
    	// XBOX controller
    	/*if(driveJoystick0.getRawButton(1))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowHomeSwitchHeight);
    	}
    	else if(driveJoystick0.getRawButton(2))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowPickupToteHeight); // Pick up a tote by rasing 4 in
    	}
    	else if(driveJoystick0.getRawButton(4))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowRaiseToteToStack); // Pick up a tote by rasing 4 in
    	}
    	else if(driveJoystick0.getRawButton(3))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowStackingToteOnTote);
    	}
    	else if(driveJoystick0.getRawButton(8))
    	{
    		driver = Driver.NARROW_DRIVER;
    	}*/
    	
    	/*if(driveJoystick0.getRawButton(3))
    	{
    		speed = -1.0;
    		//narrowLift.GoToPosition(RobotConfiguration.narrowHomeSwitchHeight);
    	}
    	else if(driveJoystick0.getRawButton(5))
    	{
    		speed = 1.0;
    		//narrowLift.GoToPosition(RobotConfiguration.narrowPickupToteHeight); // Pick up a tote by rasing 4 in
    	}
    	else
    	{
    		speed = 0.0;
    	}*/
    	/*else if(driveJoystick0.getRawButton(4))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowRaiseToteToStack); // Pick up a tote by rasing 4 in
    	}
    	else if(driveJoystick0.getRawButton(6))
    	{
    		narrowLift.GoToPosition(RobotConfiguration.narrowStackingToteOnTote);
    	}
    	else if(driveJoystick0.getRawButton(1))
    	{
    		driver = Driver.NARROW_DRIVER;
    	}*/
    	
    	// Enable a slide lock for sliding without turning
    	/*if(driveJoystick0.getRawButton(2) && driver == Driver.NARROW_DRIVER)
    	{
    		slideLock = true;
    	}
    	else if(driver == Driver.NARROW_DRIVER)
    	{
    		slideLock = false;
    	}*/
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
    	updateSmartDashboard();
    	//drive.PrintTest();
    	// Call test print statements to execute every cycle
    	//System.out.println(GenerateLiftTestString("Narrow", narrowLift));
    	//System.out.println(GenerateLiftTestString("Wide", wideLift));
    	//System.out.println(drive.GetWheelAngleString());
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    	//System.out.println("X: " + driveJoystick1.getX() + ", Y: " + driveJoystick1.getY() + ", twist: " + driveJoystick1.getTwist());

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
    	
    	double maxSpeedScale = 0.7;// Limit the speed because we know we can't actually reach motor free-running speed
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
    	
    	// OPEN LOOP
    	// Left front
    	/*drive.AddWheel(new Wheel(-halfTrack, halfWheelbase, leftWheelAxisX, 0, rollerAngle1,
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
    	*/
    	
    	// CLOSED LOOP
    	    	
    	double Kp = RobotConfiguration.wheelKp;
    	double Ki = RobotConfiguration.wheelKi;
    	
    	drive.AddWheel(new Wheel(-halfTrack, halfWheelbase, leftWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, 1.0,
    			maxWheelSpeed, RobotConfiguration.leftFrontMotorID, Kp, Ki, RobotConfiguration.leftFrontWheelEncoderPPR));
    	
    	// Right front
    	drive.AddWheel(new Wheel(halfTrack, halfWheelbase, rightWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, 1.0, maxWheelSpeed, RobotConfiguration.rightFrontMotorID, Kp, Ki, RobotConfiguration.rightFrontWheelEncoderPPR));
    	
    	// Left rear
    	drive.AddWheel(new Wheel(-halfTrack, -halfWheelbase, leftWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, 1.0,
    			maxWheelSpeed, RobotConfiguration.leftRearMotorID, Kp, Ki, RobotConfiguration.leftBackWheelEncoderPPR));
    	
    	// Right rear
    	drive.AddWheel(new Wheel(halfTrack, -halfWheelbase, rightWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, 1.0, maxWheelSpeed, RobotConfiguration.rightRearMotorID, Kp, Ki, RobotConfiguration.rightBackWheelEncoderPPR));
    	
    	// Add acceleration limits
    	drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

    	// Set the deadband for the joysticks
    	drive.SetDeadband(0.1);
    	drive.SetMinimumOutput(0.2);
    	
    	drive.Initialize();
    
    	}
    }
    
