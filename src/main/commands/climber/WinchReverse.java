package main.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class WinchReverse extends Command implements Constants{
	private boolean fastClimb;
		
	public WinchReverse(boolean fastClimb) {
        requires(Robot.cl);    
        this.fastClimb = fastClimb;
    }
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		if(fastClimb)
			Robot.cl.spin(climberMotorForwardFast * -1);
		else
			Robot.cl.spin(climberMotorForwardSlow * -1);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		//if(Robot.robotState != Robot.RobotState.Driving)
    		//Robot.robotState = Robot.RobotState.Neither;
		return true;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}
	
}