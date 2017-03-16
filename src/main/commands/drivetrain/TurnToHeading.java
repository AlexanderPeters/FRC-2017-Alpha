package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;
import main.Robot;

/**
 *
 */
public class TurnToHeading extends InstantCommand {

	private double heading;
	private double tolerance;
	
	//@param heading: the desired angle to go to (+ or - (right turn, left turn; respectively)), tolerance: the absolute difference allowable 
    public TurnToHeading(double heading, double tolerance) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.turnToHeading(heading, tolerance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.dt.getTurningPIDOnTarget();
    	/*if(Math.abs(heading - Robot.dt.getGyro().getYaw()) <= tolerance) {
    		System.out.println("True and Finished at Heading");
    		return true;

    	}
    	else
    		return false;*/
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
