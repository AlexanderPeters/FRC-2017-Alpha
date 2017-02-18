package main.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

public class FlyWheelForward extends Command{
	private double speed;

	public FlyWheelForward(double speed) {
		this.speed = speed;
		requires(Robot.shooter);
	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.robotState != Robot.RobotState.Climbing)
    		Robot.shooter.speed(speed);
    	else if(speed == -1)
    		Robot.shooter.speed();
    	else
    		Robot.shooter.speed(0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
	


}
