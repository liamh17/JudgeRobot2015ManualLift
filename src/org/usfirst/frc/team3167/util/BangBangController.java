package org.usfirst.frc.team3167.util;

// libWPI imports
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Relay;

/**
 * Class for closing a control loop using bang-bang control.  The control signal
 * output from this object has three states: 1, 0, or -1.
 *
 * @author K. Loux
 */
public class BangBangController
{
	// Fields
	private final Jaguar jag;
	private final CANJaguar canJag;
	private final Relay relay;

	private final double tolerance;
	private final boolean reverseCmd;

	private double speed = 1.0;

	// Methods
	/**
	 * Constructor for driving a Relay object.
	 *
	 * @param _relay		Specifies the relay object to which the control
	 * signal is applied
	 * @param _tolerance	Tolerance on the error signal (deadband) - control
	 * signal is zero if error magnitude is less than tolerance
	 * @param _reverseCmd	Flag indicating that the control signal should be
	 * reversed from the standard convention
	 */
	public BangBangController(Relay _relay, double _tolerance,
			boolean _reverseCmd)
	{
		relay = _relay;
		canJag = null;
		jag = null;

		tolerance = _tolerance;
		reverseCmd = _reverseCmd;
	}

	// Jaguar constructor
	/**
	 * Constructor for driving a Jaguar object.
	 *
	 * @param _jag			Jaguar object to which the control signal is applied
	 * @param _tolerance	Tolerance on the error signal (deadband) - control
	 * signal is zero if error magnitude is less than tolerance
	 * @param _reverseCmd	Flag indicating that the control signal should be
	 * reversed from the standard convention
	 */
	public BangBangController(Jaguar _jag, double _tolerance,
			boolean _reverseCmd)
	{
		jag = _jag;
		canJag = null;
		relay = null;

		tolerance = _tolerance;
		reverseCmd = _reverseCmd;
	}
	
	/**
	 * Constructor for driving a CANJaguar object.
	 *
	 * @param _jag			Jaguar object to which the control signal is applied
	 * @param _tolerance	Tolerance on the error signal (deadband) - control
	 * signal is zero if error magnitude is less than tolerance
	 * @param _reverseCmd	Flag indicating that the control signal should be
	 * reversed from the standard convention
	 */
	public BangBangController(CANJaguar _canJag, double _tolerance,
			boolean _reverseCmd)
	{
		canJag = _canJag;
		jag = null;
		relay = null;

		tolerance = _tolerance;
		reverseCmd = _reverseCmd;
	}

	public void SetSpeed(double speed)
	{
		this.speed = speed;
	}
	
	/**
	 * Method for sending a signal to Jaguars on open control loop
	 * 
	 * @param cmd	setpoint
	 */
	public void DoControl(double cmd)
	{
		if(canJag != null)
		{
			System.out.println(cmd);
			canJag.set(cmd);
		}
		else if(jag != null)
		{
			jag.set(cmd);
		}
	}

	/**
	 * Method for closing the control loop.
	 *
	 * @param cmd	Reference signal (setpoint)
	 * @param act	Feedback signal (measured response)
	 */
	public void DoControl(double cmd, double act)
	{
		// Handle relay version first
		if (relay != null)
		{
			if (cmd - act > tolerance)
			{
				relay.set(Relay.Value.kForward);
			}
			else if (cmd - act < -tolerance)
			{
				relay.set(Relay.Value.kReverse);
			}
			else
			{
				relay.set(Relay.Value.kOff);
			}
		}
		else if (jag != null)
		{
			if (cmd - act > tolerance)
			{
				if (reverseCmd)
				{
					jag.set(-speed);
				}
				else
				{
					jag.set(speed);
				}
			}
			else if (cmd - act < -tolerance)
			{
				if (reverseCmd)
				{
					jag.set(speed);
				}
				else
				{
					jag.set(-speed);
				}
			}
			else
			{
				jag.set(0.0);
			}
		}
		else if (canJag != null)
		{
			if (cmd - act > tolerance)
			{
				if (reverseCmd)
				{
					canJag.set(-speed);
				}
				else
				{
					canJag.set(speed);
				}
			}
			else if (cmd - act < -tolerance)
			{
				if (reverseCmd)
				{
					canJag.set(speed);
				}
				else
				{
					canJag.set(-speed);
				}
			}
			else
			{
				canJag.set(0.0);
			}
		}
	}
}