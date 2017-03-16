package main;

import edu.wpi.first.wpilibj.command.Command;
import lib.LatchedBoolean;
//import edu.wpi.first.wpilibj.buttons.Button;
//import lib.LatchedBoolean;
import lib.joystick.XboxController;
import main.commands.climber.WinchForward;
import main.commands.climber.WinchOff;
import main.commands.drivetrain.DriveDistance;
import main.commands.drivetrain.TurnToHeading;
//import main.commands.driverCam.SwitchCamera;
import main.commands.gearmech.GearDown;
import main.commands.gearmech.GearUp;
import main.commands.gearmech.ShiftGearMech;
import main.commands.gearmech.ToggleGearMech;
import main.commands.hood.AngleHood;
import main.commands.hood.MoveToAngle;
//import main.commands.climber.WinchForward;
//import main.commands.climber.WinchReverse;
//import main.commands.gearmech.GearMechLiftDown;
//import main.commands.gearmech.GearMechLiftUp;
//import main.commands.intake.IntakeForward;
//import main.commands.intake.IntakeOff;
import main.commands.intake.IntakeForward;
import main.commands.intake.IntakeOff;
//import main.commands.pnuematics.Shift;
import main.commands.pnuematics.ShiftDown;
import main.commands.pnuematics.ShiftUp;
import main.commands.pnuematics.ToggleCompressor;
import main.commands.shooter.FlyWheelForward;
import main.commands.shooter.FlyWheelOff;
import main.commands.stirrer.Stir;
import main.commands.vision.TurnToTarget;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Constants, HardwareAdapter {
	LatchedBoolean bool = new LatchedBoolean();
	
	//private static LatchedBoolean boole = new LatchedBoolean();
	//private static LatchedBoolean boole2 = new LatchedBoolean();
	public OI() {
		check();
	}
	
	public static XboxController getXbox (){
		return xbox;
		
	}

	public void check(){
		xbox.leftBumper.whenPressed(new ShiftUp());
		xbox.leftBumper.whenReleased(new ShiftDown());
		//xbox.start.whenPressed(new ToggleCompressor());
		xbox.rightBumper.whenPressed(new GearDown());
		xbox.rightBumper.whenReleased(new GearUp());
		//xbox.leftTrigger.whenPressed(new TurnToTarget());
		
		//xbox.rightBumper.tp
			
		
		xbox.a.whileHeld(new IntakeForward());
		xbox.a.whenReleased(new IntakeOff());
		
		//xbox.x.whenPressed(new Stir(Constants.stirrerMotorOff));
		//xbox.b.whenPressed(new Stir(Constants.stirrerMotorOff));
		xbox.x.whileHeld(new WinchForward(true));
		xbox.b.whileHeld(new WinchForward(false));
		xbox.b.whenReleased(new WinchOff());
		xbox.y.whileHeld(new FlyWheelForward(shooterForward));
		xbox.y.whileHeld(new Stir(stirrerMotorReverse));
		xbox.y.whenReleased(new FlyWheelOff());
		xbox.y.whenReleased(new Stir(0));
		xbox.rightTrigger.whileHeld(new FlyWheelForward(0.8));
		xbox.rightTrigger.whileHeld(new Stir(stirrerMotorReverse));
		xbox.rightTrigger.whenReleased(new FlyWheelOff());
		xbox.rightTrigger.whenReleased(new Stir(0));
	
		//if(DriveDistance.getfinished())
		//xbox.select.whenPressed(new DriveDistance(6, 0.42));//1/2 inch average tolerance
		//xbox.start.whenPressed(new TurnToHeading(45, 0.5f));
		//xbox.start.whenReleased(new TurnToHeading(0, 0.5));
		
		//xbox.select.whenPressed(new AngleHood(6));
		//xbox.start.whenPressed(new AngleHood(-6));
		//if(xbox.a.equals(true))
			//System.out.println(DOH);

		
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

