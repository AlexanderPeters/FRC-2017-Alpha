package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;
import main.commands.stirrer.Stir;

public class Stirrer extends Subsystem implements Constants, HardwareAdapter {
	public Stirrer() {
		
	}
	
	public void spin(double spin) {
		stirrerMotor.set(spin);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Stir());
		// TODO Auto-generated method stub
		
	}

}
