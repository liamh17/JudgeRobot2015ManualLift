package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.control.RobotPositionEstimator;

public class PickUpToteTask extends TaskBase
{
	// Fields
	private static enum State
	{
		SearchForTarget,
		LineUpToTarget,
		MoveToPickupPosition,
		LiftTote
	}
	
	private HolonomicRobotDrive drive;
	private Lift lift;
	private MotionSubTask positioner;
	private RobotPositionEstimator posEstimator;
	private final RobotKinematics lineUpPosition;
	private final RobotKinematics liftPosition;
	
	private static final double positionTolerance = 1;// tolerance for determining if we go to the next state
	
	public PickUpToteTask(HolonomicRobotDrive drive, Lift lift,
			RobotPositionEstimator posEstimator, String toteType,
			RobotKinematics lineUpPosition, RobotKinematics liftPosition)
	{
		super("Pick Up " + toteType + " Tote");
		this.drive = drive;
		this.lift = lift;
		this.lineUpPosition = lineUpPosition;
		this.liftPosition = liftPosition;
		SetNextState(State.SearchForTarget.ordinal());
	}
	
	protected boolean IsComplete()
	{
		if (GetState() == State.LiftTote.ordinal() && lift.IsLifted())
			return true;
		return false;
	}
	
	protected void EnterState()
	{
		if (GetState() == State.LineUpToTarget.ordinal())
		{
			positioner = new MotionSubTask(drive, posEstimator, lineUpPosition, true);
		}
		else if (GetState() == State.MoveToPickupPosition.ordinal())
		{
			positioner = new MotionSubTask(drive, posEstimator, liftPosition, true);
		}
		else if (GetState() == State.LiftTote.ordinal())
			lift.GoToPosition(15);// TODO:  Choose correct value, or better yet, make a static final variable
	}
	
	protected void ProcessState()
	{
		if (GetState() == State.SearchForTarget.ordinal())
		{
			if (posEstimator.SeesTarget())
				SetNextState(State.LineUpToTarget.ordinal());
		}
		else if (GetState() == State.LineUpToTarget.ordinal() ||
			GetState() == State.MoveToPickupPosition.ordinal())
		{
			positioner.Update();
			if (!posEstimator.SeesTarget())
				SetNextState(State.SearchForTarget.ordinal());
			else if (positioner.CloseToEndPosition(positionTolerance))
				SetNextState(GetState() + 1);
		}
	}
	
	protected void OnRemoveTask()
	{
		// Nothing required here
	}
	
	protected boolean OkToDrive()
	{
		return GetState() == State.SearchForTarget.ordinal();
	}
	
	public String GetStateName()
	{
		if (GetState() == State.SearchForTarget.ordinal())
			return "Search For Target";
		else if (GetState() == State.LineUpToTarget.ordinal())
			return "Line Up To Target";
		else if (GetState() == State.MoveToPickupPosition.ordinal())
			return "Move To Pickup Position";
		else if (GetState() == State.LiftTote.ordinal())
			return "Lift Tote";

		return "Unknown state";
	}
}
