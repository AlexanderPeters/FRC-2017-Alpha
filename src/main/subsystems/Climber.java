package main.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import main.Constants;

public class Climber extends Subsystem implements Constants {
	public static Spark climberMotor = new Spark(Constants.Climber_Motor);
	public static Climber instance;
	
	public static Climber getInstance() {
		if(instance == null) {
			instance = new Climber();
		}
			return instance;
	}
	
	public static void spin(double speed){
		climberMotor.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}
