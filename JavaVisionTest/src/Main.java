import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.vision.LogoTracker;

import edu.wpi.first.wpilibj.vision.AxisCamera;


public class Main
{
	public static void main(String [] args)
	{
		AxisCamera camera = new AxisCamera();
		LogoTracker tracker = new LogoTracker(camera, "/home/kerry/Projects/FIRST/JudgeRobot2015/JavaVisionTest/train.jpg");
		tracker.doAnalysis();
		if (tracker.seesTote())
		{
			System.out.println("Sees tote!");
			RobotKinematics k = tracker.getPosition();
			System.out.println("x = " + k.x);
			System.out.println("y = " + k.y);
			System.out.println("theta = " + k.theta);
		}
	}
}
