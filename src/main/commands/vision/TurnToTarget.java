package main.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class TurnToTarget extends Command implements Constants{

	public TurnToTarget() {
        requires(Robot.dt);
    }
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		if(Robot.robotState != Robot.RobotState.Climbing && Robot.comms.getTargetFound())
			Robot.dt.turnToAngle(Robot.comms.getBearing(), Robot.kToleranceDegreesDefault);
		
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
		
	}
}