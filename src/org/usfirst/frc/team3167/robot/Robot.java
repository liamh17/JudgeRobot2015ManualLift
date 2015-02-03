
package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	// TODO:  Create all the stuff here
    }
    
    private void DoCommonUpdates()
    {
    	// TODO:  Update all of our common objects
    	// Both lifts
    	// Both target trackers
    	// Holonomic drive object
    	// Task manager
    	// Anything else?
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	DoCommonUpdates();
    	// TODO:  Do stuff here
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	DoCommonUpdates();
    	// TODO:  Do stuff here
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
    }
    
}
