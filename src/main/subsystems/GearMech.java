package main.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;

public class GearMech extends Subsystem implements Constants {
	public static Solenoid gearPiston = new Solenoid(Gear_Piston);
	public static GearMech instance;
	
	public static GearMech getInstance() {
		if(instance == null) {
			instance = new GearMech();
		}
			return instance;
	}
	
	public static void lift(boolean lift){
		gearPiston.set(lift);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}