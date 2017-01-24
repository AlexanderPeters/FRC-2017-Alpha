package main.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;

public class Intake extends Subsystem implements Constants {
	/**********************
	 * Instance Data      *
	 **********************/
	public Spark intakeMotor = new Spark(Constants.Intake_Motor);
	public static Intake instance;
	//getInstance method so the constructor only runs once
	public static Intake getInstance() {
		if(instance == null) {
			instance = new Intake();
		}
			return instance;
	}
	
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
