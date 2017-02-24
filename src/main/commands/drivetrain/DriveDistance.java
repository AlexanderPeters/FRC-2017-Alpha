package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

/**
 *
 */
public class DriveDistance extends Command {

	private double distance;
	private int tolerance;
	
	//@param distance: the desired distance to go travel (+ or - (forward, backward; respectively)), tolerance: the absolute difference allowable 
    public DriveDistance(double distance, int tolerance) {
    	requires(Robot.dt);
    	this.distance = distance;
    	this.tolerance = tolerance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.resetEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.dt.driveDisplacement(distance, tolerance);
    	Robot.dt.driveStraight(-0.5);
		System.out.println("LEFT " + Robot.dt.getDistanceTraveledLeft());
		System.out.println("RIGHT " + Robot.dt.getDistanceTraveledRight());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Math.abs(Robot.dt.getDistanceTraveledLeft() + tolerance) >= distance){//Check this later
    		return true;
    	}else
    		return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
