package main.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem{
	public static DriveTrain instance;
	
	public DriveTrain getInstance() {
		if(instance == null) {
			instance = new DriveTrain();
		}
			return instance;
	}

	public void drive() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
