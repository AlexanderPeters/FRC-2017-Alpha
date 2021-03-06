package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import main.Robot;

public class TimedDriveWithBearing extends TimedCommand {
	private double throttle, bearing;
	
    public TimedDriveWithBearing(double throttle, double bearing, double time) {
    	super(time);
    	this.throttle = throttle;
    	this.bearing = bearing;
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }           

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.driveVelocity(throttle, bearing);
    }
    // Make this return true when this Command no longer needs to run execute()

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
