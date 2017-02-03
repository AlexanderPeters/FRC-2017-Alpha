package controllers;

import main.Constants;
import main.Robot;

public class DriveTrainAutonomousController implements Constants{
	private double leftVelocity, leftPosition;
	private double rightVelocity, rightPosition;
	private double heading;
	
	public void setLeftTargets(double velocity, double position) {
		this.leftVelocity = velocity;
		this.leftPosition = position;
	}
	
	public void setRightTargets(double velocity, double position) {
		this.rightVelocity = velocity;
		this.rightPosition = position;		
	}
	
	public void setHeadingTargets(double heading) {
		this.heading = heading;		
	}
	
	private void doTheMath() {
		double left = ((leftVelocity - Robot.dt.getLeftEncoderVelocity()) * leftWheelVelocityKP) + ((leftPosition - Robot.dt.getLeftEncoderPosition()) * leftWheelPositionKP);
		//double right = (stepRightVel - getRightWheelVelocity()) * K_vel) + (stepRightPos - getRightWheelPos()) * K_p);	
		//double heading = (stepHeading - gyro.getAngle()) * K_heading;
	}
	
	public double getAdjustedLeftVelocity() {
		return 0;
	}
	
	public double getAdjustedRightVelocity() {
		return 0;
	}
}
