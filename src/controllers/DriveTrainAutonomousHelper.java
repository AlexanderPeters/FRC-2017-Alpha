package controllers;

import main.Constants;
import main.Robot;

public class DriveTrainAutonomousHelper implements Constants{
	private double leftVelocityTarget, leftPositionTarget;
	private double rightVelocityTarget, rightPositionTarget;
	private double headingTarget;
	private double leftVelocityOut, rightVelocityOut;
	
	public DriveTrainAutonomousHelper() {
		Robot.dt.resetGyro();
	}
	
	public void setLeftTargets(double velocity, double position) {
		this.leftVelocityTarget = velocity;
		this.leftPositionTarget = position;
	}
	
	public void setRightTargets(double velocity, double position) {
		this.rightVelocityTarget = velocity;
		this.rightPositionTarget = position;		
	}
	
	public void setHeadingTargets(double heading) {
		this.headingTarget = heading;		
	}
	
	private void doTheMath() {
		double left = ((leftVelocityTarget - Robot.dt.getLeftEncoderVelocity()) * leftWheelVelocityKP) + ((leftPositionTarget - Robot.dt.getLeftEncoderPosition()) * leftWheelPositionKP);
		double right = ((rightVelocityTarget - Robot.dt.getRightEncoderVelocity()) * rightWheelVelocityKP) + ((rightPositionTarget - Robot.dt.getRightEncoderPosition()) * rightWheelPositionKP);	
		double heading = (headingTarget - Robot.dt.getGyro().getYaw()) * headingControllerKP;
		
		leftVelocityOut = left + heading;
		rightVelocityOut = right - heading;
	}
	
	public double getAdjustedLeftVelocity() {
		return 0;
	}
	
	public double getAdjustedRightVelocity() {
		return 0;
	}
}
