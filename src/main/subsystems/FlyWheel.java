package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;
import main.HardwareAdapter;

public class FlyWheel extends Subsystem implements Constants, HardwareAdapter{

	public void speed(Double speed) {
		shooter.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		//ToDo
		
	}

}
