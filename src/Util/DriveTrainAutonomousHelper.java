package Util;

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
	//im gay
	
	public void doTheMath() {
		double left = leftVelocityTarget / 5.85;// - Robot.dt.getLeftVelocity())/leftVelocityTarget); //* leftWheelVelocityKP);// + ((leftPositionTarget - Robot.dt.getDistanceTraveledLeft()) * leftWheelPositionKP);
		double right = rightVelocityTarget / 5.85;//- Robot.dt.getRightVelocity())/rightVelocityTarget); //* rightWheelVelocityKP);// + ((rightPositionTarget - Robot.dt.getDistanceTraveledRight()) * rightWheelPositionKP);	
		double heading = (headingTarget - Robot.dt.getGyro().getYaw()) * headingControllerKP;
		
		leftVelocityOut = left;// + heading; 
		rightVelocityOut = right;// - heading;
	}
	
	public double getAdjustedLeftVelocity() {
		return leftVelocityOut;
	}
	
	public double getAdjustedRightVelocity() {
		return rightVelocityOut;
	}
}
