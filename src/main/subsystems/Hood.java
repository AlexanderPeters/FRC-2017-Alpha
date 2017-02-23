package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class Hood extends Subsystem implements Constants, HardwareAdapter{

	public double angle = 0;

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(null);
		
	}
	
	public void set(double angle){
		hoodServo.setAngle(angle);
	}
	
	public void setI(){
		hoodServo.setAngle(angle);
	}
	
	public double get() {
		return hoodServo.getAngle();
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
