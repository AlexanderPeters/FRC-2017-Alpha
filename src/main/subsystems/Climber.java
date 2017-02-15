package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;

public class Climber extends Subsystem implements Constants, HardwareAdapter {
	
	public void spin(double speed){
		Robot.robotState = Robot.RobotState.Climbing;
		climberMotor.set(speed);
		//System.out.println("Draw 1: " + pdp.getCurrent(10) + " Draw 2: " + pdp.getCurrent(11));
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}
