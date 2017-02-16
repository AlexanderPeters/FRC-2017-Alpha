package main.commands.driverCam;

import edu.wpi.first.wpilibj.command.Command;
import main.Robot;

public class SwitchCamera extends Command {

    public SwitchCamera() {
    	requires(Robot.cc);
    }

    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.cc.switchCamera();

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