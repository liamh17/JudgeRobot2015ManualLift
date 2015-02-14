
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.TaskManager;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.Wheel;
import org.usfirst.frc.team3167.util.Conversions;
import org.usfirst.frc.team3167.util.DigitalSwitch;
import org.usfirst.frc.team3167.util.JoystickButton;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
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
			RobotConfiguration.narrowHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference);
	private Lift wideLift = new Lift(RobotConfiguration.wideToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.wideToteHomeChannel, true),
			RobotConfiguration.wideHomeSwitchHeight, RobotConfiguration.sprocketPitchCircumference);
	
	// TODO:  Add cameras and trackers
	private TaskManager taskManager = new TaskManager();
	private Joystick driveJoystick = new Joystick(1);
	
	private double testSig = 0.0;
	
	private CANJaguar jag;
	Preferences prefs = Preferences.getInstance();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	testSig = prefs.getDouble("testSig", 0.0);
    	//prefs.putDouble("Kp", 1.0);
    	//prefs.putDouble("Ki", 0.0);
    	//BuildRobotDrive();
    	// TODO:  Create all the stuff herej
    	jag = new CANJaguar(6);
    	jag.setPercentMode(CANJaguar.kQuadEncoder, 360);
    }
    
    private void DoCommonUpdates()
    {
    	//updateSmartDashboard();
    	jag.set(0.4);
    	/*taskManager.DoCurrentTask();
    	if (taskManager.OkToDrive())
    	{
    		try
    		{
    			drive.Drive(0.0, testSig, 0.0);
    		}
    		catch (Exception ex)
    		{
    			System.out.println("Failed to drive: " + ex.getMessage());
    		}
    	}*/
    	
    	/*System.out.println(narrowLift.getSwitch().IsPressed() +  " " 
    			+ narrowLift.getSwitch().HasJustBeenPressed());*/
    	
    	//narrowLift.GoToPosition(1000.0);
    	/*narrowLift.Update();
    	narrowLift.SetCmdPosition(30.0);*/
    	//System.out.println(wideLift.GetPosition());
    	//wideLift.Update();
    	//wideLift.SetCmdPosition(15.0);
    	//System.out.println("IsPressed(): " + wideLift.getSwitch().IsPressed() + ", HasJustBeenPressed(): " + wideLift.getSwitch().HasJustBeenPressed());
    	
    	// TODO:  Update all of our common objects
    	// Both target trackers
    	// Anything else?
    	
    	// CAN objects - this line sends the commands
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    }
    
    public void updateSmartDashboard()
    {
    	SmartDashboard.putNumber("Left Front Wheel KP", drive.GetWheel(0).getKP());
    	SmartDashboard.putNumber("Left Front Wheel KI", drive.GetWheel(0).getKI()); 
    	SmartDashboard.putNumber("Left Front Wheel KD", drive.GetWheel(0).getKD()); 
    	SmartDashboard.putNumber("testSig", testSig); 
    	
    	SmartDashboard.putNumber("Left Front Wheel speed", drive.GetWheel(0).GetWheelVelocity());
    	SmartDashboard.putNumber("Left Front Wheel command speed", drive.GetWheel(0).getCANMotor().get() / drive.GetWheel(0).getGearRatio() * Conversions.RPMToRadPerSec);
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
    	// TODO:  Respond to button presses, etc.
    	// Test code to move lift
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
    	/*System.out.println("X = " + driveJoystick.getX());
    	System.out.println("Y = " + driveJoystick.getY());
    	System.out.println("Z = " + driveJoystick.getTwist());*/
    	// Call test print statements to execute every cycle
    	//System.out.println(GenerateLiftTestString("Narrow", narrowLift));
    	System.out.println(GenerateLiftTestString("Wide", wideLift));
    	//System.out.println(drive.GetWheelAngleString());
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
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
    	double gearRatio = 1;// 1 because the encoder rotates with the wheel, not the motor
    	
    	// Left front
    	drive.AddWheel(new Wheel(-halfTrack, halfWheelbase, leftWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, gearRatio,
    			maxWheelSpeed, RobotConfiguration.leftFrontMotorID,
    			prefs.getDouble("Kp", 1.0), prefs.getDouble("Ki", 0.0), RobotConfiguration.leftFrontWheelEncoderPPR));
    	
    	// Right front
    	drive.AddWheel(new Wheel(halfTrack, halfWheelbase, rightWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, gearRatio,
    			maxWheelSpeed, RobotConfiguration.rightFrontMotorID, 
    			/*RobotConfiguration.wheelKp*/ 0.0, RobotConfiguration.wheelKi, RobotConfiguration.rightFrontWheelEncoderPPR));
    	
    	// Left rear
    	drive.AddWheel(new Wheel(-halfTrack, -halfWheelbase, leftWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, gearRatio,
    			maxWheelSpeed, RobotConfiguration.leftRearMotorID,
    			/*RobotConfiguration.wheelKp*/ 0.0, RobotConfiguration.wheelKi, RobotConfiguration.leftBackWheelEncoderPPR));
    	
    	// Right rear
    	drive.AddWheel(new Wheel(halfTrack, -halfWheelbase, rightWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, gearRatio,
    			maxWheelSpeed, RobotConfiguration.rightRearMotorID,
    			/*RobotConfiguration.wheelKp*/ 0.0, RobotConfiguration.wheelKi, RobotConfiguration.rightBackWheelEncoderPPR));
    	
    	// Add acceleration limits
    	drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

    	// Set the deadband for the joysticks
    	drive.SetDeadband(0.1);// horizontal deadband
    	drive.SetMinimumOutput(0.1);// vertical deadband
    	
    	drive.Initialize();
    }
    
}
