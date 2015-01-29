
package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Gyro;
import org.usfirst.frc.team3167.drive.HolonomicDrive;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{

    private Gyro gyro;
    private HolonomicDrive driveField;
    private HolonomicDrive driveRobot;
    private Joystick driver;
		
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
        gyro = new Gyro(Config.gyroChannel);
        driveField = new HolonomicDrive(Config.frequency, gyro);
        driveRobot = new HolonomicDrive(Config.frequency);
        driver = new Joystick(Config.joystickPort);
        
        initHolonomicDrive();
    }
    
    /**
     * Function to initialize the holonomic drive train class
     */ 
     private void initHolonomicDrive(HolonomicDrive drive)
     {
         // Add front left wheel
         drive.addWheel(Config.frontLeftX, Config.frontLeftY, 0.0, 45.0, Config.frontLeftCh, Config.frontleftEncChA, 
             Config.frontLeftEncB, false, Config.encPulsesPerRev, 1.0, Config.radius, Config.maxRotation, Config.frequency,
             Config.frontLeftKp, Config.frontLeftKi, Config.frontLeftKd, Config.maxIntegral, 1.0, Config.cutoff);
         
         // Add front right wheel
	 drive.addWheel(Config.frontRightX, Config.frontRightY, 0.0, 135.0, Config.frontRightCh, Config.frontRightEncChA, 
             Config.frontRightEncB, false, Config.encPulsesPerRev, 1.0, Config.radius, Config.maxRotation, Config.frequency,
             Config.frontRightKp, Config.frontRightKi, Config.frontRightKd, Config.maxIntegral, 1.0, Config.cutoff);
	 
	 // Add back right wheel
	 drive.addWheel(Config.backRightX, Config.backRightY, 0.0, 45.0, Config.backRightCh, Config.radius, 
	     Config.maxRotation);
	 
	 // Add back left wheel
	 drive.addWheel(Config.backLeftX, Config.backLeftY, 0.0, 135.0, Config.backLeftCh, Config.radius, 
	     Config.maxRotation);
             
        // Call the init method
        try
        {
            drive.init();
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.err.println("Holonomic Drive initialization has failed!");
        }
     }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	// Drive the robot based off of joystick readings
    	try
    	{
            drive.drive(driver.getX() * Config.maxVel, -driver.getY() * Config.mayVel, driver.getTwist * Config.maxRotationSpeed);
    	}
    	catch(Exception e)
    	{
    	    e.printStackTrace();
    	    System.err.println("The drive train is not responsive!");
    	}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
