package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.Robot;

public class TurnToAngle extends InstantCommand {

	private double heading;
	private double tolerance;
	private double KP, KI, KD;
	
	//@param heading: the desired angle to go to (+ or - (right turn, left turn; respectively)), tolerance: the absolute difference allowable 
    @SuppressWarnings("deprecation")
	public TurnToAngle(double heading, double tolerance) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    	this.KP = SmartDashboard.getDouble("Turning KP", 0.0);
    	this.KI = SmartDashboard.getDouble("Turning KI", 0.0);
    	this.KD = SmartDashboard.getDouble("Turning KD", 0.0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.turnToAngleSetPID(KP, KI, KD);
    	Robot.dt.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.dt.turnToAngle(heading, tolerance);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
		return Math.abs(heading - Robot.dt.getGyro().getYaw()) <= tolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
