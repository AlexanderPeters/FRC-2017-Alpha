package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class Stirrer extends Subsystem implements Constants, HardwareAdapter {
	public Stirrer() {
		
	}
	
	public void spin(double spin) {
		stirrerMotor.set(spin);
	}

	@Override
	protected void initDefaultCommand() {
		
	}
}
