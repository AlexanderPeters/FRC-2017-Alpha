package main.commands.shooter;

import edu.wpi.first.wpilibj.command.TimedCommand;
import main.Constants;
import main.Robot;

public class FlyWheelForTime extends TimedCommand implements Constants {
	private double speed;

	public FlyWheelForTime(double speed, double time) {
		super(time);
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
   }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
