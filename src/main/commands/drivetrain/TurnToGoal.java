package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.Constants;
import main.Robot;

public class TurnToGoal extends Command implements Constants {
	private double KP, KI, KD, heading, maxV, tolerance;
	private boolean cancelCommand = false;
	
	@SuppressWarnings("deprecation")
	public TurnToGoal() {
		requires(Robot.dt);
		this.KP = SmartDashboard.getDouble("Turning KP Small Angle", 0.0);
    	this.KI = SmartDashboard.getDouble("Turning KI Small Angle", 0.0);
    	this.KD = SmartDashboard.getDouble("Turning KD Small Angle", 0.0);
    	this.maxV = SmartDashboard.getDouble("Turning MaxVoltage Small Angle", 0.0);
    	this.tolerance = SmartDashboard.getDouble("Turning Tolerance", 0.0);
	}
	// Called just before this Command runs the first time
    protected void initialize() {
    	Robot.dt.turnToSmallAngleSetPID(KP, KI, KD, maxV);
    	Robot.dt.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	this.heading = Robot.comms.getBearing();
    	if(heading >= -180 && heading <= 180) {
    		System.out.println("Turning To" + heading);
    		Robot.dt.turnToSmallAngle(heading, tolerance);
    	}
    	else {
    		System.out.println("Turn To Target Called With ILLEGAL ANGLE !!!!");
    		cancelCommand = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Math.abs(heading - Robot.dt.getGyro().getYaw()) <= tolerance || cancelCommand;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
