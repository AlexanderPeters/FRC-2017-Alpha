package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import main.OI;
import main.Robot;

/**
 *
 */
public class TimedDrive extends TimedCommand {
	
	private double speed;
	
    public TimedDrive(double speed, double time) {
    	super(time);
    	this.speed = speed;
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }           

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.driveTeleop(speed, 0);//OI.getXbox().getSmoothedAltX());
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