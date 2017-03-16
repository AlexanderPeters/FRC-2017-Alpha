package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

public class DriveDistance extends Command {

	private double distance;
	private double tolerance;
	
	//@param distance: the desired distance to go travel (+ or - (forward, backward; respectively)), tolerance: the absolute difference allowable 
    public DriveDistance(double distance, double tolerance) {//feet, feet
    	requires(Robot.dt);
    	this.distance = -distance;
    	this.tolerance = tolerance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.resetSensors();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.driveDisplacement(distance, tolerance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return Robot.dt.getDistancePIDOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
