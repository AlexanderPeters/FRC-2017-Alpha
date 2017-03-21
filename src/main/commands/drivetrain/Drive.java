package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import main.OI;
import main.Robot;

/**
 *
 */
public class Drive extends Command {

    public Drive() {
    	requires(Robot.dt);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.driveVelocity(OI.getXbox().getMainY(), -OI.getXbox().getSmoothedMainX());//OI.getXbox().getSmoothedAltX());
    	//System.out.println(OI.getXbox().getMainX());
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//if(Robot.robotState != Robot.RobotState.Climbing)
    		//Robot.robotState = Robot.RobotState.Neither;
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