package main.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class WinchReverse extends Command implements Constants{
	private double speed;
	
	public WinchReverse(double speed) {
        requires(Robot.cl);
        this.speed = speed;
    }
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		Robot.cl.spin(speed * -1);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		if(Robot.robotState != Robot.RobotState.Driving)
    		Robot.robotState = Robot.RobotState.Neither;
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