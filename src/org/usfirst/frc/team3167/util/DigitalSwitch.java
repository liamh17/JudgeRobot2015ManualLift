package org.usfirst.frc.team3167.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSwitch extends Button
{
	private final DigitalInput di;
	private boolean reversed;
	
	public DigitalSwitch(int channel, boolean reversed)
	{
		di = new DigitalInput(channel);
		this.reversed = reversed;
	}
	
	protected boolean GetState()
	{
		return di.get() == !reversed;
	}
	
	public int getChannel()
	{
		return di.getChannel();
	}
}
