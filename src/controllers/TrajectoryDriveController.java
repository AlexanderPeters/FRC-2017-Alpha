package controllers;

import Util.DriveTrainAutonomousHelper;
import autoModes.TestAroundAirShip;
import lib.Loop;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;
import main.commands.gearmech.ShiftGearMech;

public class TrajectoryDriveController implements Loop, Constants, HardwareAdapter{
	private int index = 0;
	private double[][] headingArray;
	private double[][] leftVelocityArray;
	private double[][] rightVelocityArray;
	private double[][] leftPositionArray;
	private double[][] rightPositionArray;
	public static double startLeftX;
	public static double startLeftY;
	public static double startRightX;
	public static double startRightY;
	public static double voltageStart;
	private DriveTrainAutonomousHelper helper = new DriveTrainAutonomousHelper();
	@Override
	public void onStart() {
		// TODO Auto-generated method stub
		this.headingArray = TestAroundAirShip.heading;
		this.leftVelocityArray = TestAroundAirShip.leftVelocity;
		this.rightVelocityArray = TestAroundAirShip.rightVelocity;
		this.leftPositionArray = TestAroundAirShip.leftPath;
		this.rightPositionArray = TestAroundAirShip.rightPath;
		startLeftX = leftPositionArray[0][0];
		startLeftY = leftPositionArray[0][1];
		startRightX = rightPositionArray[0][0];
		startRightY = rightPositionArray[0][1];
		voltageStart = pdp.getVoltage();
		Robot.dt.resetSensors();
		
	}
	@Override
	public void onLoop() {
		// TODO Auto-generated method stub
		if(index < headingArray.length-9) {
			helper.setHeadingTargets(headingArray[index][1]);
			helper.setLeftTargets(leftVelocityArray[index][1]);
			helper.setRightTargets(rightVelocityArray[index][1]);
			helper.setLeftPositionTarget(leftPositionArray[index][0], leftPositionArray[index][1]);
			helper.setRightPositionTarget(rightPositionArray[index][0], rightPositionArray[index][1]);
			helper.doTheMath();
			Robot.dt.driveLooperControl(helper.getAdjustedLeftVelocity(), helper.getAdjustedRightVelocity());
			//System.out.println("hereatauto");

		}
		else if(index < headingArray.length-7)
			Robot.dt.driveLooperControl(0.5, 0.5);
		else if(index < headingArray.length-5)
			Robot.dt.driveLooperControl(0.25, 0.25);
		else if(index < headingArray.length-3)
			Robot.dt.driveLooperControl(0.125, 0.125);
		else if(index < headingArray.length - 1)
			Robot.dt.driveLooperControl(0.0625, 0.0625);
		else if(index < headingArray.length)
			Robot.dt.driveLooperControl(0,0);
		else if(index < headingArray.length + 10){
			new GearDown(); 
			Robot.dt.driveTeleop(-0.1, 0);
				
		}
		else {
			Robot.dt.driveTeleop(0, 0);
			new GearUp();
		}
		index++;


		
	}
	@Override
	public void onStop() {
		//new ShiftGearMech(OFF);
		// TODO Auto-generated method stub
		//No-Op
		
	}

}
