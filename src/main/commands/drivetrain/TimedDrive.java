package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import main.OI;
import main.Robot;

/**
 *
 */
public class TimedDrive extends TimedCommand {
	
	private double speed, bearing;
	
	public TimedDrive(double speed, double bearing, double time) {
    	super(time);
    	this.speed = speed;
    	this.bearing = bearing;
    	requires(Robot.dt);
    }
	
    public TimedDrive(double speed, double time) {
    	super(time);
    	this.speed = speed;
    	this.bearing = 0.0;
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }           

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.driveVelocity(speed, bearing);
    	//Robot.dt.driveStraight(speed);//OI.getXbox().getSmoothedAltX());
    	//System.out.println(OI.getXbox().getMainX());
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