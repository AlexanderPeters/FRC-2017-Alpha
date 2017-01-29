package main;

//import edu.wpi.first.wpilibj.buttons.Button;
//import lib.LatchedBoolean;
import lib.joystick.XboxController;
//import main.commands.climber.WinchForward;
//import main.commands.climber.WinchReverse;
//import main.commands.gearmech.GearMechLiftDown;
//import main.commands.gearmech.GearMechLiftUp;
import main.commands.gearmech.GearMechToggleState;
//import main.commands.intake.IntakeForward;
//import main.commands.intake.IntakeOff;
import main.commands.intake.IntakeForward;
//import main.commands.pnuematics.Shift;
import main.commands.pnuematics.ShiftDown;
import main.commands.pnuematics.ShiftUp;
import main.commands.pnuematics.ToggleCompressor;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Constants, HardwareAdapter {
	
	//private static LatchedBoolean boole = new LatchedBoolean();
	//private static LatchedBoolean boole2 = new LatchedBoolean();
	
	public static XboxController getXbox (){
		return xbox;
		
	}

	public void check(){
		xbox.leftJoystickButton.whenPressed(new ShiftUp());
		xbox.leftJoystickButton.whenReleased(new ShiftDown());
		xbox.start.whenPressed(new ToggleCompressor());
		xbox.rightBumper.whenPressed(new GearMechToggleState());
		xbox.a.whileHeld(new IntakeForward());
		if(xbox.a.equals(true))
			System.out.println(doh);

		//xbox.start.whenPressed(new ());
		//xbox.select.whenPressed(new SwitchCamera());
		// Bumpers
		/*boole.setInput(xbox.leftBumper.get());
		if(boole.getOutput())
			new IntakeForward();
		
		if(!boole.getOutput())
			new IntakeOff();
		
		
		boole2.setInput(xbox.rightBumper.get());
		if(boole2.getOutput())
			new GearMechLiftDown();
		
		if(!boole2.getOutput())
			new GearMechLiftUp();*/
		
		
	}
}

