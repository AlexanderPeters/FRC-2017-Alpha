package controllers;

import Util.DriveTrainAutonomousHelper;
import autoModes.TestAroundAirShip;
import lib.Loop;
import main.Robot;

public class TrajectoryDriveController implements Loop{
	private int index = 0;
	private double[][] headingArray;
	private double[][] leftVelocityArray;
	private double[][] rightVelocityArray;
	private double[][] leftPositionArray;
	private double[][] rightPositionArray;
	private DriveTrainAutonomousHelper helper = new DriveTrainAutonomousHelper();
	@Override
	public void onStart() {
		// TODO Auto-generated method stub
		this.headingArray = TestAroundAirShip.heading;
		this.leftVelocityArray = TestAroundAirShip.leftVelocity;
		this.rightVelocityArray = TestAroundAirShip.rightVelocity;
		this.leftPositionArray = TestAroundAirShip.leftPath;
		this.rightPositionArray = TestAroundAirShip.rightPath;
		
	}
	@Override
	public void onLoop() {
		// TODO Auto-generated method stub
		if(headingArray.length -1 <= index) {
			helper.setHeadingTargets(headingArray[index][1]);
			helper.setLeftTargets(leftVelocityArray[index][1], leftPositionArray[index][1]);
			helper.setRightTargets(rightVelocityArray[index][1], rightPositionArray[index][1]);
			helper.doTheMath();
			Robot.dt.driveLooperControl(helper.getAdjustedLeftVelocity(), helper.getAdjustedRightVelocity());
			index++;

		}

		
	}
	@Override
	public void onStop() {
		// TODO Auto-generated method stub
		//No-Op
		
	}

}
