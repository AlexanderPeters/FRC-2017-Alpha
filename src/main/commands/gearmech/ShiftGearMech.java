
package main.commands.gearmech;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

/**
 *
 */
public class ShiftGearMech extends Command implements Constants{

	private DoubleSolenoid.Value v;
    public ShiftGearMech(DoubleSolenoid.Value v) {
    	requires(Robot.pn);
    	this.v = v;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.robotState != Robot.RobotState.Climbing)
    		Robot.pn.shiftGearMech(v);
    	
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
