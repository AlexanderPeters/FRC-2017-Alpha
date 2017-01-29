package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class Intake extends Subsystem implements Constants, HardwareAdapter {
	/*******************
	 * COMMAND METHODS *
	 * @param speed    *
	 *******************/
	public void spin(double speed){
		intakeMotor.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}
