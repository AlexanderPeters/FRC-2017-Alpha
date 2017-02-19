package main.commands.hood;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

/**
 *
 */
public class MoveToAngle extends Command {
	
	private int angle;
	
	/**
	 * Positive => Up
	 * Negative => Down
	 * @param angle
	 */
    public MoveToAngle(int angle) {
    	requires(Robot.hd);
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.hd.set(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.hd.getSwitch();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.hd.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
