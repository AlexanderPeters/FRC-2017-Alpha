package main;

import edu.wpi.first.wpilibj.buttons.Button;
import lib.LatchedBoolean;
import lib.joystick.XboxController;
import main.commands.climber.WinchForward;
import main.commands.climber.WinchReverse;
import main.commands.gearmech.GearMechLiftDown;
import main.commands.gearmech.GearMechLiftUp;
import main.commands.intake.IntakeForward;
import main.commands.intake.IntakeOff;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Constants{
	private static XboxController xbox = new XboxController(Constants.Xbox_Port);
	private static LatchedBoolean boole = new LatchedBoolean();
	private static LatchedBoolean boole2 = new LatchedBoolean();
	
	
	
	
	public static XboxController getXbox (){
		return xbox;
		
	}

	public void check(){
		//xbox.start.whenPressed(new ());
		//xbox.select.whenPressed(new SwitchCamera());
		// Bumpers
		boole.setInput(xbox.leftBumper.get());
		if(boole.getOutput())
			new IntakeForward();
		
		if(!boole.getOutput())
			new IntakeOff();
		
		
		boole2.setInput(xbox.rightBumper.get());
		if(boole2.getOutput())
			new GearMechLiftDown();
		
		if(!boole2.getOutput())
			new GearMechLiftUp();
		
		
	}
}

