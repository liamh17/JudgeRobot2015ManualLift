
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
		
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
        gyro = new Gyro(Config.gyroChannel);
        driveField = new HolonomicDrive(Config.frequency, gyro);
        driveRobot = new HolonomicDrive(Config.frequency);
        
        initHolonomicDrive();
    }
    
    /**
     * Function to initialize the holonomic drive train class
     */ 
     private void initHolonomicDrive(HolonomicDrive drive)
     {
         // Add front left wheel
         drive.addWheel(Config.frontLeftX, Config.frontLeftY, 0.0, 45.0, Config.frontLeftCh, Config.frontleftEncChA, 
             Config.frontLeftEncB, false, Config.encPulsesPerRev, 1.0, );
         
         // Add front right wheel
	 drive.addWheel();
	 
	 // Add back left wheel
	 drive.addWheel();
	 
	 // Add back right wheel
	 drive.addWheel();
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
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
