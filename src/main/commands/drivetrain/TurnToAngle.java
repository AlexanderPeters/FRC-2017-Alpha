package main.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.Constants;
import main.Robot;

public class TurnToAngle extends Command implements Constants {
	private boolean bigAngle;
	private double heading;
	private double tolerance;
	private int switchAngle;
	private double KP, KI, KD, maxV;
	
	//TODO-Fix @param headers
	//@param heading: the desired angle to go to (+ or - (right turn, left turn; respectively)), tolerance: the absolute difference allowable 
	@SuppressWarnings("deprecation")
	public TurnToAngle(double heading, double tolerance, double maxV, int switchAngle, double KP, double KI, double KD) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    	this.maxV = maxV;
    	this.KP = KP;
    	this.KI = KI;
    	this.KD = KD;
    	bigAngle = (Math.abs(heading) > turnInPlaceControllerSwitchAngle ? true:false);
    	this.switchAngle = switchAngle;
	}
	@SuppressWarnings("deprecation")
	public TurnToAngle(double heading, double tolerance, double maxV, double KP, double KI, double KD) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    	this.maxV = maxV;
    	this.KP = KP;
    	this.KI = KI;
    	this.KD = KD;
    	bigAngle = (Math.abs(heading) > turnInPlaceControllerSwitchAngle ? true:false);
    	this.switchAngle = SmartDashboard.getInt("Turn In Place Controller Switch Angle", 45);
	}
	@SuppressWarnings("deprecation")
	public TurnToAngle(double heading, double tolerance, double maxV) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    	this.maxV = maxV;
    	bigAngle = (Math.abs(heading) > turnInPlaceControllerSwitchAngle ? true:false);
    	this.KP = SmartDashboard.getDouble((bigAngle?"Turning KP Big Angle":"Turning KP Small Angle"), 0.0);
    	this.KI = SmartDashboard.getDouble((bigAngle?"Turning KI Big Angle":"Turning KI Small Angle"), 0.0);
    	this.KD = SmartDashboard.getDouble((bigAngle?"Turning KD Big Angle":"Turning KD Small Angle"), 0.0);
    	this.switchAngle = SmartDashboard.getInt("Turn In Place Controller Switch Angle", 45);
    	
    }
    @SuppressWarnings("deprecation")
	public TurnToAngle(double heading, double tolerance) {
    	requires(Robot.dt);
    	this.heading = heading;
    	this.tolerance = tolerance;
    	bigAngle = (Math.abs(heading) > turnInPlaceControllerSwitchAngle ? true:false);
    	
    	this.KP = SmartDashboard.getDouble((bigAngle?"Turning KP Big Angle":"Turning KP Small Angle"), 0.0);
    	this.KI = SmartDashboard.getDouble((bigAngle?"Turning KI Big Angle":"Turning KI Small Angle"), 0.0);
    	this.KD = SmartDashboard.getDouble((bigAngle?"Turning KD Big Angle":"Turning KD Small Angle"), 0.0);
    	this.maxV = SmartDashboard.getDouble((bigAngle?"Turning MaxVoltage Big Angle":"Turning MaxVoltage Small Angle"), 0.0);
    	this.switchAngle = SmartDashboard.getInt("Turn In Place Controller Switch Angle", 45);


    }
    
    @SuppressWarnings("deprecation")
	public TurnToAngle(double heading) {
    	requires(Robot.dt);
    	this.heading = heading;
    	bigAngle = (Math.abs(heading) > turnInPlaceControllerSwitchAngle ? true:false);
    	
    	this.KP = SmartDashboard.getDouble((bigAngle?"Turning KP Big Angle":"Turning KP Small Angle"), 0.0);
    	this.KI = SmartDashboard.getDouble((bigAngle?"Turning KI Big Angle":"Turning KI Small Angle"), 0.0);
    	this.KD = SmartDashboard.getDouble((bigAngle?"Turning KD Big Angle":"Turning KD Small Angle"), 0.0);
    	this.maxV = SmartDashboard.getDouble((bigAngle?"Turning MaxVoltage Big Angle":"Turning MaxVoltage Small Angle"), 0.0);
    	this.switchAngle = SmartDashboard.getInt("Turn In Place Controller Switch Angle", 45);
    	this.tolerance = SmartDashboard.getDouble("Turning Tolerance", 0.0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(bigAngle)
    		Robot.dt.turnToBigAngleSetPID(KP, KI, KD, maxV);
    	else
    		Robot.dt.turnToSmallAngleSetPID(KP, KI, KD, maxV);
    	Robot.dt.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(bigAngle)
    		Robot.dt.turnToBigAngle(heading, tolerance);
    	else
    		Robot.dt.turnToSmallAngle(heading, tolerance);
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
