package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class Hood extends Subsystem implements Constants, HardwareAdapter{

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(null);
		
	}
	
	public void set(double angle){
		hoodServo.set(angle);
	}
	
	public void disable() {
		hoodServo.setDisabled();
	}
	
	public boolean getSwitch() {
		if(hoodSwitch.get())
			return false;
		else
			return true;
	}

}
