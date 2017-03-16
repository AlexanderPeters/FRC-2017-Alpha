package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;
import main.Robot;

public class Climber extends Subsystem implements Constants, HardwareAdapter {

	public void spin(double speed) {
		if (speed != 0.0)
			Robot.robotState = Robot.RobotState.Climbing;

		else if (Robot.robotState != Robot.RobotState.Driving)
			Robot.robotState = Robot.RobotState.Neither;
		climberMotor.set(speed);
		// System.out.println("Draw 1: " + pdp.getCurrent(10) + " Draw 2: " +
		// pdp.getCurrent(11));
	}

	@Override
	protected void initDefaultCommand() {

	}
}
