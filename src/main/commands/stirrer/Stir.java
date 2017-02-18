package main.commands.stirrer;

import edu.wpi.first.wpilibj.command.Command;
import main.Constants;
import main.Robot;

public class Stir extends Command implements Constants{
	//private double speed;
	
	/*public Stir(double speed) {
    	requires(Robot.str);
    	this.speed = speed;
    }*/
	public Stir() {
		requires(Robot.str);
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println(Robot.robotState);
    	if(Robot.robotState != Robot.RobotState.Climbing)
    		Robot.str.spin(stirrerMotorOn);
    	else
    		Robot.str.spin(0);
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
