package main.subsystems;

import edu.wpi.first.wpilibj.Spark;
import main.Constants;

public class Intake implements Constants {
	/**********************
	 * Instance Data      *
	 **********************/
	public Spark intakeMotor = new Spark(Constants.Intake_Motor);
	public static Intake instance;
	//getInstance method so the constructor only runs once (maybe don't need? -ryan)
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
	
}
