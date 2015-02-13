package org.usfirst.frc.team3167.drive;

/**
 * Class for position control of an axis driven by a motor.
 * Includes bang-bang control with encoder feedback and homing functionality.
 * 
 * @author kloux
 *
 */

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team3167.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.util.BangBangController;
import org.usfirst.frc.team3167.util.DigitalSwitch;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

public class BangBangAxis
{
	private enum HomeState
	{
		NotStarted,
		FastTowardSwitch,
		SlowAwayFromSwitch,
		SlowTowardSwitch,
		Homed
	}
	private HomeState homeState = HomeState.NotStarted;
	
	private final double homingSpeed;// [units/sec]
	private final double homingAccel;// [units/sec^2]
	private final DigitalSwitch homeSwitch;
	
	private final double normalSpeed;// [units/sec]
	private final double normalAccel;// [units/sec^2]
	
	private final double positionScale;// [units/encoder pulse]
	private final double homeSwitchPosition;// [units]
	private double position;
	private double cmdPosition;
	private double bufferedCmdPosition;
	
	private SecondOrderLimiter cmdGenerator;
	private BangBangController controller;
	
	private final Jaguar motor;
	private final Encoder encoder;
	private CANJaguar canMotor;
	
	private final double normalStep;// [units]
	private final double homingStep;// [units]
	
	private double initPosition;
				
	public BangBangAxis(Jaguar motor, double normalSpeed, double normalAccel,
			double homingSpeed, double homingAccel, DigitalSwitch homeSwitch,
			double positionScale, double homeSwitchPosition, double tolerance, boolean reverse)
	{
		this.motor = motor;
		this.normalSpeed = normalSpeed;
		this.normalAccel = normalAccel;
		this.homingSpeed = homingSpeed;
		this.homingAccel = homingAccel;
		this.homeSwitch = homeSwitch;
		this.positionScale = positionScale;
		this.homeSwitchPosition = homeSwitchPosition;
		bufferedCmdPosition = homeSwitchPosition;
		
		controller = new BangBangController(motor, tolerance, reverse);
		controller.SetSpeed(1.0);// Not actually speed, this just says we want to allow maximum command to jag if necessary
		
		encoder = null;// TODO:  Fix this!  If not using CAN Jag this won't work
		
		canMotor = null;
		
		normalStep = CalculateStep(normalSpeed);
		homingStep = CalculateStep(homingSpeed);
	}
	
	public BangBangAxis(CANJaguar canMotor, double normalSpeed, double normalAccel,
			double homingSpeed, double homingAccel, DigitalSwitch homeSwitch,
			double positionScale, double homeSwitchPosition, double tolerance, boolean reverse)
	{
		this.canMotor = canMotor;
		this.normalSpeed = normalSpeed;
		this.normalAccel = normalAccel;
		this.homingSpeed = homingSpeed;
		this.homingAccel = homingAccel;
		this.homeSwitch = homeSwitch;
		this.positionScale = positionScale;
		this.homeSwitchPosition = homeSwitchPosition;
		bufferedCmdPosition = homeSwitchPosition;
		
		controller = new BangBangController(canMotor, tolerance, reverse);
		controller.SetSpeed(1.0);// Not actually speed, this just says we want to allow maximum command to jag if necessary
		
		this.motor = null;
		encoder = null;
		
		normalStep = CalculateStep(normalSpeed);
		homingStep = CalculateStep(homingSpeed);
	}
	
	public HomeState getState()
	{
		return homeState;
	}
	
	private double CalculateStep(double velocity)
	{
		return velocity / RobotConfiguration.frequency;
	}
	
	public DigitalSwitch getSwitch()
	{
		return homeSwitch;
	}
	
	public void Update()
	{
		double cmdVel = 0.0;
		//double cmdVel = (cmdPosition - GetPosition()) * RobotConfiguration.frequency;
		System.out.println("CmdVel: " + cmdVel + ", Position: " + GetPosition());
		if (homeState != HomeState.Homed)
		{
			switch (homeState)
			{
			default:
			case NotStarted:
				System.out.println(homeState);
				cmdVel = 0.0;
				cmdPosition = GetPosition();
				cmdGenerator = new SecondOrderLimiter(normalSpeed, normalAccel, RobotConfiguration.frequency);
				homeState = HomeState.FastTowardSwitch;
				
			case FastTowardSwitch:
				System.out.println(homeState);
				cmdVel = -normalStep;
				if (homeSwitch.IsPressed())
				{
					cmdGenerator = new SecondOrderLimiter(homingSpeed, homingAccel, RobotConfiguration.frequency);
					homeState = HomeState.SlowAwayFromSwitch;
				}
				break;
				
			case SlowAwayFromSwitch:
				System.out.println(homeState);
				cmdVel = homingStep;
				if (!homeSwitch.IsPressed())
					homeState = HomeState.SlowTowardSwitch;
				break;
				
			case SlowTowardSwitch:
				System.out.println(homeState);
				if (homeSwitch.HasJustBeenPressed())
				{
					initPosition = GetPosition();
					homeState = HomeState.Homed;
					position = homeSwitchPosition;
					cmdPosition = bufferedCmdPosition;
					cmdGenerator = new SecondOrderLimiter(normalSpeed, normalAccel, RobotConfiguration.frequency);
				}
				else
					cmdVel = -homingStep;
				break;
				
			}
			controller.DoControl(cmdVel*RobotConfiguration.frequency/normalSpeed);
		}
		else
		{
			//System.out.println("Post-state machine cmdVel: " + cmdVel);	
			//cmdVel = cmdGenerator.Process(cmdVel);// TODO:  Fix this!
			//System.out.println("Post-state processing cmdVel: " + cmdVel);	
			//cmdPosition += cmdVel/RobotConfiguration.frequency;
			//System.out.println("cmdPosition: " + cmdPosition);
			if(!controller.inRange())
			{
				controller.SetSpeed(1.0);
				controller.DoControl(cmdPosition, GetPosition());
			}
			else
			{
				controller.SetSpeed(0.25);
				controller.DoControl(cmdPosition, GetPosition());
			}
		}
		
	}
	
	public double GetPosition()
	{
		if(homeState != HomeState.Homed)
		{
			if (motor != null)
			{
				return encoder.getRaw() * positionScale;
			}
			else if (canMotor != null)
			{
				return canMotor.getPosition() * positionScale;
			}
			return 0.0;
		}
		else
		{
			if (motor != null)
			{
				return (encoder.getRaw() * positionScale) - (initPosition) 
						+ homeSwitchPosition;
			}
			else if (canMotor != null)
			{
				return (canMotor.getPosition() * positionScale) - initPosition 
						+ homeSwitchPosition;
			}
			return 0.0;
		}
	}

	
	public void SetCmdPosition(double cmdPosition)
	{
		if (homeState == HomeState.Homed)
			this.cmdPosition = cmdPosition;
		else
			bufferedCmdPosition = cmdPosition;
	}
}
