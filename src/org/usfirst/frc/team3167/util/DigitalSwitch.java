package org.usfirst.frc.team3167.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSwitch extends Button
{
	private final DigitalInput di;
	private boolean reversed;
	private boolean stateRequested = false;
	
	public DigitalSwitch(int channel, boolean reversed)
	{
		di = new DigitalInput(channel);
		this.reversed = reversed;
	}
	
	protected boolean GetState()
	{
		return di.get() == !reversed;
	}
	
	public boolean HasJustBeenPressed()
	{
		// If the button is pressed, and we haven't asked for the state yet,
		// return true and reset the stateRequested flag
		if (GetState() && !stateRequested)
		{
			stateRequested = true;
			return true;
		}
		else if (!GetState())
			stateRequested = false;
		
		return false;
		
	}
	
	public boolean IsPressed()
	{
		return GetState();
 	}
	
	public int getChannel()
	{
		return di.getChannel();
	}
}
