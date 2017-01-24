package main.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class WinchOff extends Command implements Constants{

	public WinchOff() {
        requires(Robot.cl);
    }
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		Robot.cl.spin(0);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
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