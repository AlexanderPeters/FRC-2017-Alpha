package main.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class TurnToTarget extends Command implements Constants{

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}/*

	public TurnToTarget() {
        requires(Robot.dt);
    }
	
	@Override
	protected void initialize() {
		if(Robot.robotState != Robot.RobotState.Climbing && Robot.comms.getTargetFound())
			Robot.dt.turnToAngle(Robot.comms.getBearing(), Robot.kToleranceDegreesDefault);		
	}

	@Override
	protected void execute() {
				
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
		
	}*/
}