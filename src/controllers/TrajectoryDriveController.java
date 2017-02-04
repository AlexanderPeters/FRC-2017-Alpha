package controllers;

import autoModes.TestAroundAirShip;
import lib.Loop;

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
		/*helper.setHeadingTargets(headingArray[][index]);
		helper.setLeftTargets(leftVelocityArray[][index], rightVelocityArray[][index]);
		helper.setrightTargets(rightVelocityArray[][index], rightVelocityArray[][index]);

		index++;*/
		
	}
	@Override
	public void onStop() {
		// TODO Auto-generated method stub
		//No-Op
		
	}

}
