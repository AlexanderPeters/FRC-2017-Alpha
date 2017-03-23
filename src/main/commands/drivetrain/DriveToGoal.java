package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.Constants;
import main.Robot;

public class DriveToGoal extends Command implements Constants {
	private double KP, KI, KD, distance, maxV, tolerance;

	@SuppressWarnings("deprecation")
	public DriveToGoal() {// feet
		requires(Robot.dt);
		this.KP = SmartDashboard.getDouble("Distance KP", 0.0);
		this.KI = SmartDashboard.getDouble("Distance KI", 0.0);
		this.KD = SmartDashboard.getDouble("Distance KD", 0.0);
		this.tolerance = SmartDashboard.getDouble("Distance Tolerance", 0.0);
		this.maxV = SmartDashboard.getDouble("Distance MaxVoltage", 0.0);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.dt.driveDistanceSetPID(KP, KI, KD, maxV);
		Robot.dt.resetSensors();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		this.distance = -(Robot.comms.getRange()*Math.cos(cameraAngle * Math.PI/180)) - desiredDistanceToGoal;//Drive negated
    	System.out.println("Driving To" + distance);
		Robot.dt.driveDistance(distance, tolerance);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(distance - Robot.dt.getDistanceTraveledRight()) <= tolerance;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}

}
